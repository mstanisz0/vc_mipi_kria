/*
 * imx273.c - imx273 sensor driver
 * Based on the imx219 and ov5693 drivers
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define VC_CODE             1   /* CCC - imx273.c - enable code for VC MIPI camera  */
#define IMX273_ENB_MUTEX    0   /* CCC - imx273.c - mutex lock/unlock */
#define IMX273_TRIG_MODE    0   /* CCC - imx273.c - force sensor ext. trigger mode */
#define IMX273_TRIG_FIX     0   /* CCC - imx273.c - fix trigger mode problem:                     */
                                /* in trigger mode the sensor produces frame height = IMX273_DY-2 */
                                /* switch between frame format tables:                            */
                                /*   imx273_frmfmt      : in free-run mode                        */
                                /*   imx273_trig_frmfmt : in trigger mode                         */
#define STOP_STREAMING_SENSOR_RESET   1   /* CCC - imx273_stop_streaming() - reset sensor before streaming stop */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/debugfs.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

//#include <media/tegra_v4l2_camera.h>
#include <media/tegra-v4l2-camera.h>
#include <media/tegracam_core.h>
//#include <media/camera_common.h>

#include "../platform/tegra/camera/camera_gpio.h"
#include "vc_mipi.h"

//#include "imx273.h"
//#include "imx273_mode_tbls.h"

//#define CREATE_TRACE_POINTS
//#include <trace/events/ov5693.h>



#define IMX273_TABLE_WAIT_MS    0
#define IMX273_TABLE_END        1


// imx252
///* In dB*10 */
//#define IMX273_DIGITAL_GAIN_MIN        0x00
//#define IMX273_DIGITAL_GAIN_MAX        0x7A5     // 1957
//#define IMX273_DIGITAL_GAIN_DEFAULT    0x00
//
///* In usec */
//#define IMX273_DIGITAL_EXPOSURE_MIN     160         // microseconds (us)
//#define IMX273_DIGITAL_EXPOSURE_MAX     10000000
//#define IMX273_DIGITAL_EXPOSURE_DEFAULT 10000

// imx296:
///* In dB*10 */
#define IMX273_DIGITAL_GAIN_MIN     0
#define IMX273_DIGITAL_GAIN_MAX     480
#define IMX273_DIGITAL_GAIN_DEFAULT 20

/* In usec */
#define IMX273_DIGITAL_EXPOSURE_MIN     29          // microseconds (us)
#define IMX273_DIGITAL_EXPOSURE_MAX     15110711
#define IMX273_DIGITAL_EXPOSURE_DEFAULT 10000

// PK: just for test, not supported yet
//#define IMX273_MIN_FRAME_LENGTH     100
//#define IMX273_MAX_FRAME_LENGTH     800000
#define IMX273_FRAME_RATE_DEFAULT   30000000   // 30 fps (*1000000)

// Sensor modes:
// 0x00 :  8bit, 2 lanes, streaming
// 0x01 : 10bit, 2 lanes, streaming
// 0x02 : 12bit, 2 lanes, streaming
// 0x03 :  8bit, 2 lanes, external trigger global shutter reset
// 0x04 : 10bit, 2 lanes, external trigger global shutter reset
// 0x05 : 12bit, 2 lanes, external trigger global shutter reset
//
// 0x06 :  8bit, 4 lanes, streaming
// 0x07 : 10bit, 4 lanes, streaming
// 0x08 : 12bit, 4 lanes, streaming
// 0x09 :  8bit, 4 lanes, external trigger global shutter reset
// 0x0A : 10bit, 4 lanes, external trigger global shutter reset
// 0x0B : 12bit, 4 lanes, external trigger global shutter reset

/* VC Sensor Mode - default 10-Bit Streaming */
#if IMX273_TRIG_MODE
static int sensor_mode = 4;    /* VC sensor mode: 0-2=8/10/12-bit (2 lanes), 3-5=8/10/12-bit ext. trigger (2 lanes), 6-11=... (4 lanes) */
#else
static int sensor_mode = 1;    /* VC sensor mode: 0-2=8/10/12-bit (2 lanes), 3-5=8/10/12-bit ext. trigger (2 lanes), 6-11=... (4 lanes) */
#endif

static int ext_trig_mode = -1;      // ext. trigger mode: -1:not set from DT, >=0:set from DT
static int flash_output  = 0;       // flash output enable
static int fpga_addr = 0x10;        // FPGA i2c address (def = 0x10)


//module_param(sensor_mode, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
//MODULE_PARM_DESC(sensor_mode, "VC Sensor Mode: 0=10bit_stream 1=10bit_4lanes");

enum imx_model {
    IMX_MODEL_NONE = 0,
    IMX273_MODEL_MONOCHROME,
    IMX273_MODEL_COLOR,
//    IMX297_MODEL_MONOCHROME,
//    IMX297_MODEL_COLOR,
};


enum {
    IMX273_MODE_1440X1080 = 0,
    IMX273_MODE_START_STREAM,
    IMX273_MODE_STOP_STREAM,
};

#define imx273_reg struct reg_8

static const imx273_reg imx273_start[] = {
    {0x7000, 0x01},         /* mode select streaming on */
    {IMX273_TABLE_END, 0x00}
};

static const imx273_reg imx273_stop[] = {
    {0x7000, 0x00},         /* mode select streaming off */
    {IMX273_TABLE_END, 0x00}
};


//---------------------------- 1440 x 1080 mode -------------------------------

// 1. 10-bit: Jetson Nano/Xavier requires sensor width, multiple of 32.
// 2. 8-bit: Jetson Nano/Xavier requires sensor width, multiple of 64.

#define X0 0
#define Y0 0

#define TEST_1408   0   /* DDD - test frame width = 1048 */

#define IMX273_DX 1440
#define IMX273_DY 1080

#if TEST_1408
  #undef  IMX273_DX
  #define IMX273_DX 1408
#endif

//#if IMX273_TRIG_FIX // trig mode frame DY:
//  #define IMX273_DY 1078
//#else
//  #define IMX273_DY 1080
//#endif
//

static const imx273_reg imx273_mode_1440X1080[] = {
//#if 0
#if TEST_1408
    { 0x6015, IMX273_DX & 0xFF }, { 0x6016, (IMX273_DX>>8) & 0xFF },   // hor. output width  L,H  = 0x6015,0x6016,
    { 0x6010, IMX273_DY & 0xFF }, { 0x6011, (IMX273_DY>>8) & 0xFF },   // ver. output height L,H  = 0x6010,0x6011,
#endif
    { IMX273_TABLE_END, 0x00 }
};
#undef  X0
#undef  Y0

static const imx273_reg *imx273_mode_table[] = {
    [IMX273_MODE_1440X1080]    = imx273_mode_1440X1080,
    [IMX273_MODE_START_STREAM] = imx273_start,
    [IMX273_MODE_STOP_STREAM]  = imx273_stop,
};

static const int imx273_120fps[] = {
    60,
};

static const struct camera_common_frmfmt imx273_frmfmt[] = {
    { { IMX273_DX, IMX273_DY }, imx273_120fps, ARRAY_SIZE(imx273_120fps), 0,
      IMX273_MODE_1440X1080 },
};

//#if IMX273_TRIG_FIX   // [[[ - trig mode fix: does not work in free-run
  static const struct camera_common_frmfmt imx273_trig_frmfmt[] = {
    { { IMX273_DX, IMX273_DY-2 }, imx273_120fps, ARRAY_SIZE(imx273_120fps), 0,
      IMX273_MODE_1440X1080 },
};
//#endif  // ]]]

struct vc_rom_table {
    char   magic[12];
    char   manuf[32];
    u16    manuf_id;
    char   sen_manuf[8];
    char   sen_type[16];
    u16    mod_id;
    u16    mod_rev;
    char   regs[56];
    u16    nr_modes;
    u16    bytes_per_mode;
    char   mode1[16];
    char   mode2[16];
};

struct imx273 {
    struct i2c_client          *i2c_client;
    struct v4l2_subdev         *subdev;
    u16                         fine_integ_time;
    u32                         frame_length;
    u32                         frame_rate;
    u32                         digital_gain;
//                                u32 analog_gain;
    u32                         exposure_time;

    struct camera_common_i2c    i2c_dev;
    struct camera_common_data  *s_data;
    struct tegracam_device     *tc_dev;

//    struct mutex        streaming_lock;
    bool                streaming;

//    s32                 group_hold_prev;
//    bool                group_hold_en;

#if VC_CODE     // [[[ - new VC code
    struct i2c_client   *rom;
    struct vc_rom_table rom_table;
    struct mutex mutex;
    int    cam_mode;        // camera mode:
                            //   IMX273_MODE_1440X1080 (default)
    enum imx_model model;   // camera model
    int sensor_ext_trig;    // ext. trigger flag: 0=no, 1=yes
    int flash_output;       // flash output enable
    int sen_clk;            // sen_clk default=54Mhz imx183=72Mhz
    int sensor_mode;        // sensor mode
    int num_lanes;          // # of data lanes: 1, 2, 4

    int fpga_addr;          // FPGA i2c address (def = 0x10)
#endif  // ]]]

};

static const struct regmap_config imx273_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
    .cache_type = REGCACHE_RBTREE,
    .use_single_rw = true,
};


#if 0   // [[[
//
//struct vc_rom_table {
//    char   magic[12];
//    char   manuf[32];
//    u16    manuf_id;
//    char   sen_manuf[8];
//    char   sen_type[16];
//    u16    mod_id;
//    u16    mod_rev;
//    char   regs[56];
//    u16    nr_modes;         /*  256   modes */
//    u16    bytes_per_mode;   /*   16   bytes */
//    char   modes[4096];
//    enum vc_rom_table_mode_format  mode_format;
//    union  vc_rom_table_mode mode[VC_ROM_TABLE_NR_MODES_MAX]; /* points to  modes */
//};
//
//struct vc_sen_reg_content {
//    u16    modelId;
//    u8     chipRev;
//    u8     stateIdle;
//    u16    hStart;      /* minimum horizontal position   for current sensor mode */
//    u16    vStart;      /* minimum vertical   position   for current sensor mode */
//    u16    hSize;       /* horizontal sensor resoultion  for current sensor mode */
//    u16    vSize;       /* vertical   sensor resoultion  for current sensor mode */
//    u16    hOutSize;    /* horizontal sensor resoultion  for the MIPI interface  */
//    u16    vOutSize;    /* vertical   sensor resoultion  for the MIPI interface  */
//    u32    exposure;    /* exposure time in us */
//    u16    gain;        /* gain in 10*db */
//    u8     RESERVED1;
//    u8     RESERVED2;
//    u8     RESERVED3;
//    u8     RESERVED4;
//    u8     RESERVED5;
//    u8     RESERVED6;
//    u8     RESERVED7;
//};
//
#endif // ]]]

enum vc_rom_table_sreg_names {
    MODEL_ID_HIGH = 0,
    MODEL_ID_LOW,
    CHIP_REV,
    IDLE,
    H_START_HIGH,
    H_START_LOW,
    V_START_HIGH,
    V_START_LOW,
    H_SIZE_HIGH,
    H_SIZE_LOW,
    V_SIZE_HIGH,
    V_SIZE_LOW,
    H_OUTPUT_HIGH,
    H_OUTPUT_LOW,
    V_OUTPUT_HIGH,
    V_OUTPUT_LOW,
    EXPOSURE_HIGH,
    EXPOSURE_MIDDLE,
    EXPOSURE_LOW,
    GAIN_HIGH,
    GAIN_LOW,
    RESERVED1,
    RESERVED2,
    RESERVED3,
    RESERVED4,
    RESERVED5,
    RESERVED6,
    RESERVED7,
};

#define sen_reg(priv, addr) (*((u16 *)(&priv->rom_table.regs[(addr)*2])))


// camera_common.h
// ---------------
//struct tegracam_ctrl_ops {
//    u32 numctrls;
//    u32 string_ctrl_size[TEGRA_CAM_MAX_STRING_CONTROLS];
//    const u32 *ctrl_cid_list;
//    bool is_blob_supported;
//    int (*set_gain)(struct tegracam_device *tc_dev, s64 val);
//    int (*set_exposure)(struct tegracam_device *tc_dev, s64 val);
//    int (*set_exposure_short)(struct tegracam_device *tc_dev, s64 val);
//    int (*set_frame_rate)(struct tegracam_device *tc_dev, s64 val);
//    int (*set_group_hold)(struct tegracam_device *tc_dev, bool val);
//    int (*fill_string_ctrl)(struct tegracam_device *tc_dev,
//                struct v4l2_ctrl *ctrl);
//    int (*set_gain_ex)(struct tegracam_device *tc_dev,
//            struct sensor_blob *blob, s64 val);
//    int (*set_exposure_ex)(struct tegracam_device *tc_dev,
//            struct sensor_blob *blob, s64 val);
//    int (*set_frame_rate_ex)(struct tegracam_device *tc_dev,
//            struct sensor_blob *blob, s64 val);
//    int (*set_group_hold_ex)(struct tegracam_device *tc_dev,
//            struct sensor_blob *blob, bool val);
//};
//
// tegracam_core.h
// ---------------
//struct tegracam_device {
//    struct camera_common_data   *s_data;
//    struct media_pad        pad;
//    u32                 version;
//    bool                is_streaming;
//    /* variables to be filled by the driver to register */
//    char                name[32];
//    struct i2c_client       *client;
//    struct device           *dev;
//    u32             numctrls;
//    const u32           *ctrl_cid_list;
//    const struct regmap_config  *dev_regmap_config;
//    struct camera_common_sensor_ops     *sensor_ops;
//    const struct v4l2_subdev_ops        *v4l2sd_ops;
//    const struct v4l2_subdev_internal_ops   *v4l2sd_internal_ops;
//    const struct media_entity_operations    *media_ops;
//    const struct tegracam_ctrl_ops      *tcctrl_ops;
//    void    *priv;
//};

//tegra-v4l2-camera.h
//-------------------
//struct unpackedU64 {
//    __u32 high;
//    __u32 low;
//};
//union __u64val {
//    struct unpackedU64 unpacked;
//    __u64 val;
//};
//
//struct sensor_signal_properties {
//    __u32 readout_orientation;
//    __u32 num_lanes;
//    __u32 mclk_freq;
//    union __u64val pixel_clock;
//    __u32 cil_settletime;
//    __u32 discontinuous_clk;
//    __u32 dpcm_enable;
//    __u32 tegra_sinterface;
//    __u32 phy_mode;
//    __u32 deskew_initial_enable;
//    __u32 deskew_periodic_enable;
//    union __u64val serdes_pixel_clock;
//    __u32 reserved[2];
//};
//
//struct sensor_image_properties {
//    __u32 width;
//    __u32 height;
//    __u32 line_length;
//    __u32 pixel_format;
//    __u32 embedded_metadata_height;
//    __u32 reserved[11];
//};
//
//struct sensor_dv_timings {
//    __u32 hfrontporch;
//    __u32 hsync;
//    __u32 hbackporch;
//    __u32 vfrontporch;
//    __u32 vsync;
//    __u32 vbackporch;
//    __u32 reserved[10];
//};
//
//struct sensor_control_properties {
//    __u32 gain_factor;
//    __u32 framerate_factor;
//    __u32 inherent_gain;
//    __u32 min_gain_val;
//    __u32 max_gain_val;
//    __u32 min_hdr_ratio;
//    __u32 max_hdr_ratio;
//    __u32 min_framerate;
//    __u32 max_framerate;
//    union __u64val min_exp_time;
//    union __u64val max_exp_time;
//    __u32 step_gain_val;
//    __u32 step_framerate;
//    __u32 exposure_factor;
//    union __u64val step_exp_time;
//    __u32 default_gain;
//    __u32 default_framerate;
//    union __u64val default_exp_time;
//    __u32 reserved[10];
//};
//
//struct sensor_mode_properties {
//    struct sensor_signal_properties signal_properties;
//    struct sensor_image_properties image_properties;
//    struct sensor_control_properties control_properties;
//    struct sensor_dv_timings dv_timings;
//};


#if VC_CODE     // [[[ - new VC code

/****** reg_write = Write reg = 09.2020 *********************************/
static int reg_write(struct i2c_client *client, const u16 addr, const u8 data)
{
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg;
    u8 tx[3];
    int ret;

    msg.addr = client->addr;
    msg.buf = tx;
    msg.len = 3;
    msg.flags = 0;
    tx[0] = addr >> 8;
    tx[1] = addr & 0xff;
    tx[2] = data;

    ret = i2c_transfer(adap, &msg, 1);
    mdelay(2);

    return ret == 1 ? 0 : -EIO;
}

/****** reg_read = Read reg = 09.2020 ***********************************/
static int reg_read(struct i2c_client *client, const u16 addr)
{
    u8 buf[2] = {addr >> 8, addr & 0xff};
    int ret;
    struct i2c_msg msgs[] = {
        {
            .addr  = client->addr,
            .flags = 0,
            .len   = 2,
            .buf   = buf,
        }, {
            .addr  = client->addr,
            .flags = I2C_M_RD,
            .len   = 1,
            .buf   = buf,
        },
    };

    ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    if (ret < 0) {
        dev_warn(&client->dev, "Reading register %x from %x failed\n",
             addr, client->addr);
        return ret;
    }

    return buf[0];
}

///****** reg_write_table = Write reg table = 09.2020 *******************/
//static int reg_write_table(struct i2c_client *client,
//               const struct imx273_reg table[])
//{
//    const struct imx273_reg *reg;
//    int ret;
//
//    for (reg = table; reg->addr != IMX273_TABLE_END; reg++) {
//        ret = reg_write(client, reg->addr, reg->val);
//        if (ret < 0)
//            return ret;
//    }
//
//    return 0;
//}

#endif  // ]]] - VC_CODE

/****** imx273_read_reg = Read register = 09.2020 ***********************/
static int imx273_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val)
{
    int err = 0;
    u32 reg_val = 0;

    err = regmap_read(s_data->regmap, addr, &reg_val);
    *val = reg_val & 0xff;

    return err;
}

/****** imx273_write_reg = Write register = 09.2020 *********************/
static int imx273_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
    int err = 0;

    err = regmap_write(s_data->regmap, addr, val);
    if (err)
        dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x",
            __func__, addr, val);

    return err;
}

/****** imx273_write_table = Write table = 09.2020 **********************/
static int imx273_write_table(struct imx273 *priv, const imx273_reg table[])
{
    return regmap_util_write_table_8(priv->s_data->regmap, table, NULL, 0,
        IMX273_TABLE_WAIT_MS, IMX273_TABLE_END);
}


#define vc_mipi_common_reg_write  reg_write
#define vc_mipi_common_reg_read   reg_read

///****** vc_mipi_common_reg_read = Read reg common = 09.2020 *************/
////static int vc_mipi_common_reg_read(struct camera_common_data *s_data, u16 addr)
//int vc_mipi_common_reg_read(struct camera_common_data *s_data, u16 addr)
//{
//    int err = 0;
//    u8  val;
//
//    err = imx273_read_reg(s_data, addr, &val);
//    if(err) return -1;
//
//    return (int)val;
//}
//
///****** vc_mipi_common_reg_write = Write reg common = 09.2020 ***********/
//static int vc_mipi_common_reg_write(struct camera_common_data *s_data, u16 addr, u8 val)
//{
//    return imx273_write_reg(s_data, addr, val);
//}


/****** imx273_gpio_set = GPIO set = 09.2020 ****************************/
static void imx273_gpio_set(struct camera_common_data *s_data,
                unsigned int gpio, int val)
{
    struct camera_common_pdata *pdata = s_data->pdata;

    if (pdata && pdata->use_cam_gpio)
        cam_gpio_ctrl(s_data->dev, gpio, val, 1);
    else {
        if (gpio_cansleep(gpio))
            gpio_set_value_cansleep(gpio, val);
        else
            gpio_set_value(gpio, val);
    }
}

/****** imx273_set_gain = Set gain = 09.2020 ****************************/
static int imx273_set_gain(struct tegracam_device *tc_dev, s64 val)
{

#define TRACE_IMX273_SET_GAIN         1   /* DDD - imx273_set_gain() - trace */
#define IMX273_SET_GAIN_STOP_STREAM   0   /* CCC - imx273_set_gain() - stop streaming before set gain */
//#define IMX273_SET_GAIN_GROUP_HOLD   0   /* CCC - imx273_set_gain() - enable group hold bef reg write */

//    struct camera_common_data *s_data = tc_dev->s_data;
    struct imx273 *priv = (struct imx273 *)tc_dev->priv;
    struct device *dev = tc_dev->dev;
    struct i2c_client *client = priv->i2c_client;

//    imx273_reg regs[3];
//    s64 gain = val;
    int ret = 0;


    priv->digital_gain = val;

    if ( priv->digital_gain < IMX273_DIGITAL_GAIN_MIN)
       priv->digital_gain = IMX273_DIGITAL_GAIN_MIN;
    else if ( priv->digital_gain > IMX273_DIGITAL_GAIN_MAX)
       priv->digital_gain = IMX273_DIGITAL_GAIN_MAX;

//    gain = IMX273_DIGITAL_GAIN_DEFAULT; // VC: FIXED to 1.00 for now


//............. Stop streaming before gain change
#if IMX273_SET_GAIN_STOP_STREAM
    if(priv->streaming)
    {
      ret = imx273_write_table(priv, imx273_mode_table[IMX273_MODE_STOP_STREAM]);
      if(ret)
      {
        dev_err(dev, "%s(): imx273_write_table() err=%d\n", __func__, ret);
        return ret;
      }
    }
#endif


#if TRACE_IMX273_SET_GAIN
    dev_err(dev, "%s: Set gain = %d\n", __func__, (int)priv->digital_gain);
#endif

//............. Set IMX273 gain
    if(sen_reg(priv, GAIN_HIGH))
        ret  = vc_mipi_common_reg_write(client, sen_reg(priv, GAIN_HIGH),
            (priv->digital_gain >> 8) & 0xff);
    if(sen_reg(priv, GAIN_LOW))
        ret |= vc_mipi_common_reg_write(client, sen_reg(priv, GAIN_LOW) ,
                priv->digital_gain       & 0xff);


//
////    ret = 0;
//#if 1
//    if(sen_reg(priv, GAIN_HIGH))
//      ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, GAIN_HIGH), (priv->digital_gain >> 8) & 0xff);     // gain high
//    if(sen_reg(priv, GAIN_LOW))
//      ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, GAIN_LOW),  priv->digital_gain & 0xff);           // gain low
//#else
////0070:05 04          # gain high = 0x0405
////0072:04 04          # gain low  = 0x0404
//#define GAIN_H  0x0405
//#define GAIN_L  0x0404
//      ret |= vc_mipi_common_reg_write(s_data, GAIN_L,  priv->digital_gain & 0xff);           // gain low
//      ret |= vc_mipi_common_reg_write(s_data, GAIN_H, (priv->digital_gain >> 8) & 0xff);     // gain high
//#endif
////    mdelay(100); // wait ms


//    regs[0].addr = 0x000A;      // gain high
//    regs[0].val = (priv->digital_gain >> 8) & 0x03;
//    regs[1].addr =  0x0009;      // gain low
//    regs[1].val = priv->digital_gain & 0xff;
//    regs[2].addr = IMX273_TABLE_END;
//    regs[2].val = 0;
//
//
//
////#if IMX273_SET_GAIN_GROUP_HOLD
////    if (!priv->group_hold_prev)
////        imx273_set_group_hold(tc_dev, 1);
////#endif
//
//    ret = imx273_write_table(priv, regs);
//    if (ret)
//        goto fail;


//
//    /* If enabled, apply settings immediately */
//    reg = reg_read(client, 0x3000);
//    if ((reg & 0x1f) == 0x01)
//        imx273_s_stream(&priv->subdev, 1);

    if(ret)
      dev_err(dev, "%s: error=%d\n", __func__, ret);

//............. Start streaming after gain change
#if IMX273_SET_GAIN_STOP_STREAM
    if(priv->streaming)
    {
      ret = imx273_write_table(priv, imx273_mode_table[IMX273_MODE_START_STREAM]);
      if(ret)
      {
        dev_err(dev, "%s(): imx273_write_table() err=%d\n", __func__, ret);
        return ret;
      }
    }
#endif

//    return 0;
//
//fail:
    return ret;
}

//
// IMX296
// 1H period 14.815us
// NumberOfLines=1118
//
#define H1PERIOD_296 242726 // (U32)(14.815 * 16384.0)
#define NRLINES_296  (1118)
#define TOFFSET_296  233636 // (U32)(14.260 * 16384.0)
#define VMAX_296     1118
#define EXPOSURE_TIME_MIN_296  29
#define EXPOSURE_TIME_MIN2_296 16504
#define EXPOSURE_TIME_MAX_296  15534389


//
// IMX297
// 1H period 14.411us
// NumberOfLines=574
//
#define H1PERIOD_297 236106 // (U32)(14.411 * 16384.0)
#define NRLINES_297  (574)
#define TOFFSET_297  233636 // (U32)(14.260 * 16384.0)
#define VMAX_297    574
#define EXPOSURE_TIME_MIN_297  29
#define EXPOSURE_TIME_MIN2_297 8359
#define EXPOSURE_TIME_MAX_297  15110711


static int imx_exposure_296_297(struct imx273 *priv, s32 expMin0, s32 expMin1, s32 expMax, s32 nrLines, s32 tOffset, s32 h1Period, s32 vMax)
{
//    struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
    struct i2c_client *client = priv->i2c_client;
    int ret = 0;
    u32 exposure = 0;
    u32 base;

//    u32 denominator=priv->tpf.denominator;
//    u32 numerator=priv->tpf.numerator;
//    u32 fps=priv->cur_mode->max_fps;
    u32 multiplier = 1000;   // (fps*numerator*1000)/denominator;
    dev_info(&client->dev, "multiplier = %d \n",multiplier);

    switch(priv->model)
    {
#ifndef PLATFORM_QCOM
        case IMX273_MODEL_MONOCHROME:
        case IMX273_MODEL_COLOR:
            base = 0x0200;
            break;
#endif
        default:
            base = 0x3000;
    }

#if 1
    {
    int reg;
//    static int hmax=0;
    static int vmax=0;
    // VMAX
    //if(vmax == 0)
    {
    vmax=0;
    reg = vc_mipi_common_reg_read(client, base+0x12); // HIGH
    if(reg) vmax = reg & 0xff;
    reg = vc_mipi_common_reg_read(client, base+0x11); // MIDDLE
    if(reg) vmax = (vmax << 8) | (reg & 0xff);
    reg = vc_mipi_common_reg_read(client, base+0x10); // LOW
    if(reg) vmax = (vmax << 8) | (reg & 0xff);
    dev_info(&client->dev, "vmax = %08x \n",vmax);
    dev_info(&client->dev, "vmax = %d \n",vmax);
    }
    vMax = vmax; // TEST

#if 0
    // HMAX
    //if (hmax == 0)
    {
    reg = vc_mipi_common_reg_read(client, 0x7003); // HIGH
    if(reg) hmax = (hmax << 8) | reg;
    reg = vc_mipi_common_reg_read(client, 0x7002); // LOW
    if(reg) hmax = (hmax << 8) | reg;
    dev_info(&client->dev, "hmax = %08x \n",hmax);
    dev_info(&client->dev, "hmax = %d \n",hmax);
    }
#endif

    }
#endif

//    // do nothing in ext_trig mode
//    if (priv->cur_mode->sensor_ext_trig) return 0;

    if (priv->exposure_time < expMin0)
        priv->exposure_time = expMin0;

    if (priv->exposure_time > expMax)
        priv->exposure_time = expMax;

    //if (priv->exposure_time < expMin1*multiplier/2)
    if (priv->exposure_time < expMin1)
    {
        // exposure = (NumberOfLines - exp_time / 1Hperiod + toffset / 1Hperiod )
        ////exposure = (nrLines  -  ((u32)(priv->exposure_time) * 16384 - tOffset)/h1Period);
        exposure = (vMax  -  ((u32)(priv->exposure_time) * 16384 - tOffset)/h1Period);
        exposure = exposure - ( vMax - ( (vMax * multiplier) / 1000 )); // set frame rate
        if(exposure < 14)
            exposure = 14;

        dev_info(&client->dev, "SHS = %d vMax= %d\n",exposure,vMax);
#if 1
#if 1
        //vMax = (vMax * multiplier) / 1000; // set frame rate
        ret  = vc_mipi_common_reg_write(client, base+0x12, (vMax >> 16) & 0x07);
#else
        ret  = vc_mipi_common_reg_write(client, base+0x12, 0x00);
#endif
        ret |= vc_mipi_common_reg_write(client, base+0x11, (vMax >>  8) & 0xff);
        ret |= vc_mipi_common_reg_write(client, base+0x10,  vMax        & 0xff);
#endif
        if(sen_reg(priv, EXPOSURE_HIGH))
            ret |= vc_mipi_common_reg_write(client, sen_reg(priv, EXPOSURE_HIGH)  , (exposure >> 16) & 0x07);
        if(sen_reg(priv, EXPOSURE_MIDDLE))
            ret |= vc_mipi_common_reg_write(client, sen_reg(priv, EXPOSURE_MIDDLE), (exposure >>  8) & 0xff);
        if(sen_reg(priv, EXPOSURE_LOW))
            ret |= vc_mipi_common_reg_write(client, sen_reg(priv, EXPOSURE_LOW)   ,  exposure        & 0xff);
    }
    else
    {
        //exposure = 5 + ((u64)(priv->exposure_time) * 16384 - tOffset)/h1Period;
        u64 divresult;
        u32 divisor ,remainder;
        divresult = ((unsigned long long)priv->exposure_time * 16384) - tOffset;
        divisor   = h1Period;
        remainder = (u32)(do_div(divresult,divisor)); // caution: division result value at dividend!
#if 1
        //exposure = 5 + ((u32)divresult * multiplier) / 1000; // set frame rate
        exposure = 15 + ((u32)divresult * multiplier) / 1000; // set frame rate
#else
        //exposure = 5 + (u32)divresult;
        exposure = 15 + (u32)divresult;
#endif
        dev_info(&client->dev, "VMAX = %d \n",exposure);

        if(sen_reg(priv, EXPOSURE_HIGH))
            ret  = vc_mipi_common_reg_write(client, sen_reg(priv, EXPOSURE_HIGH), 0x00);
        if(sen_reg(priv, EXPOSURE_MIDDLE))
            ret |= vc_mipi_common_reg_write(client, sen_reg(priv, EXPOSURE_MIDDLE), 0x00);
        if(sen_reg(priv, EXPOSURE_LOW))
            //ret |= vc_mipi_common_reg_write(client, sen_reg(priv, EXPOSURE_LOW), 0x04);
            ret |= vc_mipi_common_reg_write(client, sen_reg(priv, EXPOSURE_LOW), 0x0e);

        ret |= vc_mipi_common_reg_write(client, base+0x12, (exposure >> 16) & 0x07);
        ret |= vc_mipi_common_reg_write(client, base+0x11, (exposure >>  8) & 0xff);
        ret |= vc_mipi_common_reg_write(client, base+0x10,  exposure        & 0xff);

    }

    return ret;
}

#define imx_exposure_296(priv)    imx_exposure_296_297(priv, EXPOSURE_TIME_MIN_296, EXPOSURE_TIME_MIN2_296, EXPOSURE_TIME_MAX_296, NRLINES_296 ,TOFFSET_296, H1PERIOD_296, VMAX_296)
#define imx_exposure_297(priv)    imx_exposure_296_297(priv, EXPOSURE_TIME_MIN_297, EXPOSURE_TIME_MIN2_297, EXPOSURE_TIME_MAX_297, NRLINES_297 ,TOFFSET_297, H1PERIOD_297, VMAX_297)


/****** imx273_set_exposure = Set exposure = 09.2020 ********************/
static int imx273_set_exposure(struct tegracam_device *tc_dev, s64 val)
{

//                                                          expMin0,                   expMin1,                   expMax,                 nrLines,
//static int imx_exposure_imx273(struct imx *priv, s32 EXPOSURE_TIME_MIN_273, s32 EXPOSURE_TIME_MIN2_273, s32 EXPOSURE_TIME_MAX_273, s32 NRLINES_273,
//                                                            tOffset,         h1Period,         vMax)
//                                                        s32 TOFFSET_273, s32 H1PERIOD_273, s32 VMAX_273)

#define TRACE_IMX273_SET_EXPOSURE           1   /* DDD - imx273_set_exposure() - trace */
#define DUMP_EXPOSURE_PARAMS                0   /* DDD - imx273_set_exposure() - dump DT exposure params */
#define IMX273_SET_EXPOSURE_STOP_STREAM     0   /* CCC - imx273_set_exposure() - stop streaming before set exposure */

    struct device *dev = tc_dev->dev;
    struct imx273 *priv = tc_dev->priv;
//    struct camera_common_data *s_data = priv->s_data;
//    struct imx273 *priv = (struct imx273 *)tc_dev->priv;
//    struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);

    int ret=0;
//    u32 exposure = 0;
//    int err = 0;

//    imx273_reg regs[7];


//// do nothing in ext_trig mode
//    if (priv->sensor_ext_trig) return 0;

#if IMX273_ENB_MUTEX
    mutex_lock(&priv->mutex);
#endif


#if DUMP_EXPOSURE_PARAMS
{
//    const struct sensor_mode_properties *mode =  &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
    struct sensor_mode_properties *mode =  &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

//    dev_err(dev, "%s: mode->control_properties:\n", __func__);
    dev_err(dev, "%s: min_exp_time,max_exp_time=%d,%d default_exp_time=%d\n", __func__,
         (int)mode->control_properties.min_exp_time.val,
         (int)mode->control_properties.max_exp_time.val,
         (int)mode->control_properties.default_exp_time.val);
}
#endif

    priv->exposure_time = (u32)val;

//    if(priv->exposure_time < IMX273_DIGITAL_EXPOSURE_MIN) priv->exposure_time = IMX273_DIGITAL_EXPOSURE_MIN;
//    if(priv->exposure_time > IMX273_DIGITAL_EXPOSURE_MAX) priv->exposure_time = IMX273_DIGITAL_EXPOSURE_MAX;

    if (priv->exposure_time < IMX273_DIGITAL_EXPOSURE_MIN)
        priv->exposure_time = IMX273_DIGITAL_EXPOSURE_MIN;

    if (priv->exposure_time > IMX273_DIGITAL_EXPOSURE_MAX)
        priv->exposure_time = IMX273_DIGITAL_EXPOSURE_MAX;


/*----------------------------------------------------------------------*/
/*                   Set exposure: Ext. trigger mode                    */
/*----------------------------------------------------------------------*/
    if(priv->sensor_ext_trig)
    {
        u64 exposure = (priv->exposure_time * (priv->sen_clk/1000000)); // sen_clk default=54Mhz, imx183=72Mhz
        int addr;
        int data;
        int ret = 0;

#if TRACE_IMX273_SET_EXPOSURE
        dev_err(dev, "%s(): exposure_time=%d: TRIG exposure=%llu (0x%llx)\n", __func__, priv->exposure_time, exposure, exposure);
#endif

//............. Stop streaming before exposure change
//#if IMX273_SET_EXPOSURE_STOP_STREAM
//    if(priv->streaming)
//    {
//      ret = imx273_write_table(priv, imx273_mode_table[IMX273_MODE_STOP_STREAM]);
//      if(ret)
//      {
//        dev_err(dev, "%s(): imx273_write_table() err=%d\n", __func__, ret);
//        return ret;
//      }
//    }
//#endif


/*
register 9  [0x0109]: exposure LSB (R/W, default: 0x10)
register 10 [0x010A]: exposure     (R/W, default: 0x27)
register 11 [0x010B]: exposure     (R/W, default: 0x00)
register 12 [0x010C]: exposure MSB (R/W, default: 0x00)

This is the 32bit register (4 x 8bit) for the exposure control when
using the fast trigger mode.
The exposure counter is using the internal 74.25MHz clock, i.e.
the exact exposure value is calculated as follows:

   exposure_time [nsec] = exposure_register[31:0] * 13.4680nsec

Make sure to write registers 9-12 LSB first, MSB last.
Writing the MSB will transfer the complete exposure value into
a buffer register which will update the internal exposure
counter as soon as the current exposure has finished.
*/

//        addr = 0x0108; // ext trig enable
//        // data =      1; // external trigger enable
//        // data =      4; // TEST external self trigger enable
//        data = priv->sensor_ext_trig; // external trigger enable
//        ret = reg_write(priv->rom, addr, data);
//
//        addr = 0x0103; // flash output enable
//        data =      1; // flash output enable
//        ret = reg_write(priv->rom, addr, data);

        addr = 0x0109; // shutter lsb
        data = exposure & 0xff;
        ret |= reg_write(priv->rom, addr, data);

        addr = 0x010a;
        data = (exposure >> 8) & 0xff;
        ret |= reg_write(priv->rom, addr, data);

        addr = 0x010b;
        data = (exposure >> 16) & 0xff;
        ret |= reg_write(priv->rom, addr, data);

        addr = 0x010c; // shutter msb
        data = (exposure >> 24) & 0xff;
        ret |= reg_write(priv->rom, addr, data);

//............. Start streaming after exposure change
//#if IMX273_SET_EXPOSURE_STOP_STREAM
//    if(priv->streaming)
//    {
//      ret = imx273_write_table(priv, imx273_mode_table[IMX273_MODE_START_STREAM]);
//      if(ret)
//      {
//        dev_err(dev, "%s(): imx273_write_table() err=%d\n", __func__, ret);
//        return ret;
//      }
//    }
//#endif

    } /* if(priv->sensor_ext_trig) */

/*----------------------------------------------------------------------*/
/*                       Set exposure: Free-run mode                    */
/*----------------------------------------------------------------------*/
    else
    {

//............. Stop streaming before exposure change
#if IMX273_SET_EXPOSURE_STOP_STREAM
      if(priv->streaming)
      {
        ret = imx273_write_table(priv, imx273_mode_table[IMX273_MODE_STOP_STREAM]);
        if(ret)
        {
          dev_err(dev, "%s(): imx273_write_table() err=%d\n", __func__, ret);
          return ret;
        }
      }
#endif

#if TRACE_IMX273_SET_EXPOSURE
      dev_err(dev, "%s: Set exposure=%d\n", __func__, priv->exposure_time);
#endif

/*............. Set IMX273 exposure */
      ret = imx_exposure_296(priv);
      if(ret)
      {
          dev_err(dev, "%s(): imx_exposure_296() err=%d\n", __func__, ret);
          return ret;
      }

/*----------------------------------------------------------------------*/
#if 1       // [[[ - imx296 compat
/*----------------------------------------------------------------------*/
{

////
//// IMX296
//// 1H period 14.815us
//// NumberOfLines=1118
////
//#define H1PERIOD_296 242726 // (U32)(14.815 * 16384.0)
//#define NRLINES_296  (1118)
//#define TOFFSET_296  233636 // (U32)(14.260 * 16384.0)
//#define VMAX_296     1118
//#define EXPOSURE_TIME_MIN_296  29
//#define EXPOSURE_TIME_MIN2_296 16504
//#define EXPOSURE_TIME_MAX_296  15534389
//
////static int                        imx_exposure_296_297(struct imx *priv, s32 expMin0,           s32 expMin1,            s32 expMax,            s32 nrLines, s32 tOffset, s32 h1Period, s32 vMax)
////#define imx_exposure_296(priv)    imx_exposure_296_297(priv,             EXPOSURE_TIME_MIN_296, EXPOSURE_TIME_MIN2_296, EXPOSURE_TIME_MAX_296, NRLINES_296 ,TOFFSET_296, H1PERIOD_296, VMAX_296)
//
//    s32 expMin0  = EXPOSURE_TIME_MIN_296;
//    s32 expMin1  = EXPOSURE_TIME_MIN2_296;
//    s32 expMax   = EXPOSURE_TIME_MAX_296;
//    s32 nrLines  = NRLINES_296;
//    s32 tOffset  = TOFFSET_296;
//    s32 h1Period = H1PERIOD_296;
//    s32 vMax     = VMAX_296;
//
//
//    if (priv->exposure_time < expMin0)
//        priv->exposure_time = expMin0;
//
//    if (priv->exposure_time > expMax)
//        priv->exposure_time = expMax;
//
//    if (priv->exposure_time < expMin1)
//    {
//        // exposure = (NumberOfLines - exp_time / 1Hperiod + toffset / 1Hperiod )
//        exposure = (nrLines  -  ((u32)(priv->exposure_time) * 16384 - tOffset)/h1Period);
////        dev_info(dev, "SHS = %d\n",exposure);
//#if TRACE_IMX273_SET_EXPOSURE
//        dev_err(dev, "%s: Set exposure=%d, SHS=%u\n", __func__, priv->exposure_time, exposure);
//#endif
//
//        ret  = vc_mipi_common_reg_write(s_data, 0x3012, 0x00);
//        ret |= vc_mipi_common_reg_write(s_data, 0x3011, (vMax >> 8) & 0xff);
//        ret |= vc_mipi_common_reg_write(s_data, 0x3010,  vMax       & 0xff);
//
//        if(sen_reg(priv, EXPOSURE_HIGH))
//            ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_HIGH)  , (exposure >> 16) & 0x07);
//        if(sen_reg(priv, EXPOSURE_MIDDLE))
//            ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_MIDDLE), (exposure >>  8) & 0xff);
//        if(sen_reg(priv, EXPOSURE_LOW))
//            ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_LOW)   ,  exposure        & 0xff);
//    }
//    else
//    {
//        //exposure = 5 + ((u64)(priv->exposure_time) * 16384 - tOffset)/h1Period;
//        u64 divresult;
//        u32 divisor ,remainder;
//        divresult = ((unsigned long long)priv->exposure_time * 16384) - tOffset;
//        divisor   = h1Period;
//        remainder = (u32)(do_div(divresult,divisor)); // caution: division result value at dividend!
//        exposure = 5 + (u32)divresult;
//
////        dev_info(dev, "VMAX = %u\n",exposure);
//#if TRACE_IMX273_SET_EXPOSURE
//        dev_err(dev, "%s: Set exposure=%d, VMAX=%u\n", __func__, priv->exposure_time, exposure);
//#endif
//
//        if(sen_reg(priv, EXPOSURE_HIGH))
//            ret  = vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_HIGH), 0x00);
//        if(sen_reg(priv, EXPOSURE_MIDDLE))
//            ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_MIDDLE), 0x00);
//        if(sen_reg(priv, EXPOSURE_LOW))
//            ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_LOW), 0x04);
//
//        ret |= vc_mipi_common_reg_write(s_data, 0x3012, (exposure >> 16) & 0x07);
//        ret |= vc_mipi_common_reg_write(s_data, 0x3011, (exposure >>  8) & 0xff);
//        ret |= vc_mipi_common_reg_write(s_data, 0x3010,  exposure        & 0xff);
//    }
//
}
/*----------------------------------------------------------------------*/
#else       // ]]] [[[ - imx252 compat
/*----------------------------------------------------------------------*/
{

////
//// IMX273
//// 1H period 20.00us
//// NumberOfLines=1080
////
//#define H1PERIOD_273 327680 // (U32)(20.000 * 16384.0)
//#define NRLINES_273  (1080)
//#define TOFFSET_273  47563  // (U32)(2.903 * 16384.0)
//#define VMAX_273     3079
//#define EXPOSURE_TIME_MIN_273  160
//#define EXPOSURE_TIME_MIN2_273 74480
//#define EXPOSURE_TIME_MAX_273  10000000
//
////006A:8F 02          # exposure H            = 0x028f
////006C:8E 02          # exposure M            = 0x028e
////006E:8D 02          # exposure L            = 0x028d
////0070:05 04          # gain high             = 0x0405
////0072:04 04          # gain low              = 0x0404
//
///*
//#define VMAX_H  0x3012  // 0x0212
//#define VMAX_M  0x3011  // 0x0211
//#define VMAX_L  0x3010  // 0x0210
//
//#define HMAX_H  0x3015  // 0x0215
//#define HMAX_L  0x3014  // 0x0214
//*/
//
//#define VMAX_H  0x0212
//#define VMAX_M  0x0211
//#define VMAX_L  0x0210
//
//#define HMAX_H  0x0215
//#define HMAX_L  0x0214
//
//    s32 vMax = VMAX_273;
//    int reg;
//    static int hmax=0;
//    static int vmax=0;
//
//      // VMAX
//      if(vmax == 0)
//      {
//        reg = vc_mipi_common_reg_read(s_data, VMAX_H); // HIGH
//        if(reg) vmax = reg;
//        reg = vc_mipi_common_reg_read(s_data, VMAX_M); // MIDDLE
//        if(reg) vmax = (vmax << 8) | reg;
//        reg = vc_mipi_common_reg_read(s_data, VMAX_L); // LOW
//        if(reg) vmax = (vmax << 8) | reg;
//        dev_err(dev, "vmax = %d (0x%08x)\n",vmax, vmax);
//      }
//      vMax = vmax; // TEST
//
//      // HMAX
//      if (hmax == 0)
//      {
//        reg = vc_mipi_common_reg_read(s_data, HMAX_H); // HIGH
//        if(reg) hmax = (hmax << 8) | reg;
//        reg = vc_mipi_common_reg_read(s_data, HMAX_L); // LOW
//        if(reg) hmax = (hmax << 8) | reg;
//        dev_err(dev, "hmax = %d (0x%08x)\n",hmax,hmax);
//      }
//
//#if TRACE_IMX273_SET_EXPOSURE
////      dev_err(dev, "%s: vMax=%d\n", __func__, vMax);
//#endif
//
//      if (priv->exposure_time < EXPOSURE_TIME_MIN2_273)
//      {
//
////Exposure time  [s] = (1 H period) * (Number of lines per frame - SHS) + 13.73 (TBD) [�s]
//// Exposure time [s] = (XTRIG low level pulse width [�s]) + 13.73 (TBD)[�s]
////: Exposure time error (tOFFSET)
//
//        // exposure = (NumberOfLines - exp_time / 1Hperiod + toffset / 1Hperiod )
//        // shutter = {VMAX - SHR}*HMAX + 209(157) clocks
//
//        //exposure = (NRLINES_273  -  ((int)(priv->exposure_time) * 16384 - TOFFSET_273)/H1PERIOD_273);
//        exposure = (vMax -  ((int)(priv->exposure_time) * 16384 - TOFFSET_273)/H1PERIOD_273);
////        dev_info(dev, "SHS = %d \n",exposure);
//
//#if TRACE_IMX273_SET_EXPOSURE
//      dev_err(dev, "%s: Set exposure = %d (< %d), vMax=%d SHS=%d\n", __func__, priv->exposure_time, EXPOSURE_TIME_MIN2_273, vMax, exposure);
//#endif
//
//        ret  = vc_mipi_common_reg_write(s_data, VMAX_H, (vMax >> 16) & 0xff);
//        ret |= vc_mipi_common_reg_write(s_data, VMAX_M, (vMax >>  8) & 0xff);
//        ret |= vc_mipi_common_reg_write(s_data, VMAX_L,  vMax        & 0xff);
//
//        if(sen_reg(priv, EXPOSURE_MIDDLE))
//          ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_MIDDLE), (exposure >>  8) & 0xff);
//        if(sen_reg(priv, EXPOSURE_LOW))
//          ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_LOW)   ,  exposure        & 0xff);
//      }
//      else
//      {
//        // exposure = 5 + ((unsigned long long)(priv->exposure_time * 16384) - TOFFSET_273)/H1PERIOD_273;
//        u64 divresult;
//        u32 divisor ,remainder;
//        divresult = ((unsigned long long)priv->exposure_time * 16384) - TOFFSET_273;
//        divisor   = H1PERIOD_273;
//        remainder = (u32)(do_div(divresult,divisor)); // caution: division result value at dividend!
//        exposure = 5 + (u32)divresult;
//
////        dev_info(dev, "VMAX = %d \n",exposure);
//#if TRACE_IMX273_SET_EXPOSURE
//      dev_err(dev, "%s: Set exposure = %d, VMAX=%d\n", __func__, priv->exposure_time, exposure);
//#endif
//
//        if(sen_reg(priv, EXPOSURE_MIDDLE))
//          ret  = vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_MIDDLE), 0x00);
//        if(sen_reg(priv, EXPOSURE_LOW))
//          ret |= vc_mipi_common_reg_write(s_data, sen_reg(priv, EXPOSURE_LOW), 0x04);
//
//        ret |= vc_mipi_common_reg_write(s_data, VMAX_H, (exposure >> 16) & 0x07);
//        ret |= vc_mipi_common_reg_write(s_data, VMAX_M, (exposure >>  8) & 0xff);
//        ret |= vc_mipi_common_reg_write(s_data, VMAX_L,  exposure        & 0xff);
//      }
}
#endif  // ]]]

//............. Start streaming after exposure change
#if IMX273_SET_EXPOSURE_STOP_STREAM
    if(priv->streaming)
    {
      ret = imx273_write_table(priv, imx273_mode_table[IMX273_MODE_START_STREAM]);
      if(ret)
      {
        dev_err(dev, "%s(): imx273_write_table() err=%d\n", __func__, ret);
        return ret;
      }
    }
#endif

    } /* else: free-run mode */

#if IMX273_ENB_MUTEX
    mutex_unlock(&priv->mutex);
#endif

    return ret;
}

/****** imx273_set_frame_rate = Set frame rate = 09.2020 ****************/
static int imx273_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{

#define TRACE_IMX273_SET_FRAME_RATE     0   /* DDD - imx273_set_frame_rate() - trace */

//    struct camera_common_data *s_data = tc_dev->s_data;
//    struct device *dev = tc_dev->dev;
//    struct imx273 *priv = tc_dev->priv;
    int err = 0;

//    u32 frame_length;
//    u32 frame_rate = (int)val;
////    int i;
//
//    frame_length = (910 * 120) / frame_rate;
//
//#if TRACE_IMX273_SET_FRAME_RATE
//    dev_err(dev, "%s: frame_rate=%d: frame_length=%d\n", __func__, (int)val,frame_length);
//#endif
//
//    if(frame_length < IMX273_MIN_FRAME_LENGTH) frame_length = IMX273_MIN_FRAME_LENGTH;
//    if(frame_length > IMX273_MAX_FRAME_LENGTH) frame_length = IMX273_MAX_FRAME_LENGTH;
//
////      err = imx273_set_frame_length(tc_dev, frame_length);
////      if (err)
////         goto fail;
//
//    priv->frame_rate = frame_rate;
//    priv->frame_length = frame_length;
//    if(err)
//    {
//      dev_err(dev, "%s: error=%d\n", __func__, err);
//    }

    return err;

}


/****** imx273_set_group_hold = Set group hold = 09.2020 ****************/
static int imx273_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
    /* imx273 does not support group hold */
    return 0;
}

// CID list
static const u32 ctrl_cid_list[] = {
    TEGRA_CAMERA_CID_GAIN,
    TEGRA_CAMERA_CID_EXPOSURE,
    TEGRA_CAMERA_CID_FRAME_RATE,
    TEGRA_CAMERA_CID_SENSOR_MODE_ID,
//    TEGRA_CAMERA_CID_EXPOSURE_SHORT,
//    TEGRA_CAMERA_CID_GROUP_HOLD,
//    TEGRA_CAMERA_CID_HDR_EN,
//    TEGRA_CAMERA_CID_EEPROM_DATA,
//    TEGRA_CAMERA_CID_OTP_DATA,
//    TEGRA_CAMERA_CID_FUSE_ID,

};


static struct tegracam_ctrl_ops imx273_ctrl_ops = {
    .numctrls = ARRAY_SIZE(ctrl_cid_list),
    .ctrl_cid_list = ctrl_cid_list,
    .set_gain = imx273_set_gain,
    .set_exposure = imx273_set_exposure,
    .set_frame_rate = imx273_set_frame_rate,  // imx273_set_frame_length,
    .set_group_hold = imx273_set_group_hold,

};

/****** imx273_power_on = Power on = 09.2020 ****************************/
static int imx273_power_on(struct camera_common_data *s_data)
{
#define TRACE_IMX273_POWER_ON   0   /* DDD - imx273_power_on() - trace */

    int err = 0;
    struct camera_common_power_rail *pw = s_data->power;
    struct camera_common_pdata *pdata = s_data->pdata;
    struct device *dev = s_data->dev;


#if TRACE_IMX273_POWER_ON
    dev_err(dev, "%s: power on\n", __func__);
#endif

    if (pdata && pdata->power_on) {
        err = pdata->power_on(pw);
        if (err)
            dev_err(dev, "%s failed.\n", __func__);
        else
            pw->state = SWITCH_ON;
        return err;
    }

// IMX219
//    if (pw->reset_gpio) {
//        if (gpio_cansleep(pw->reset_gpio))
//            gpio_set_value_cansleep(pw->reset_gpio, 0);
//        else
//            gpio_set_value(pw->reset_gpio, 0);
//    }

    if (unlikely(!(pw->avdd || pw->iovdd || pw->dvdd)))
        goto skip_power_seqn;

    usleep_range(10, 20);

    if (pw->avdd) {
        err = regulator_enable(pw->avdd);
        if (err)
            goto imx273_avdd_fail;
    }

    if (pw->iovdd) {
        err = regulator_enable(pw->iovdd);
        if (err)
            goto imx273_iovdd_fail;
    }

// IMX
//    if (pw->dvdd) {
//        err = regulator_enable(pw->dvdd);
//        if (err)
//            goto imx273_dvdd_fail;
//    }

    usleep_range(10, 20);

skip_power_seqn:
    usleep_range(1, 2);
    if (gpio_is_valid(pw->pwdn_gpio))
        imx273_gpio_set(s_data, pw->pwdn_gpio, 1);

    /*
     * datasheet 2.9: reset requires ~2ms settling time
     * a power on reset is generated after core power becomes stable
     */
    usleep_range(2000, 2010);
//    usleep_range(23000, 23100);

//    msleep(20);

    if (gpio_is_valid(pw->reset_gpio))
        imx273_gpio_set(s_data, pw->reset_gpio, 1);

//    msleep(20);

// IMX
//    if (pw->reset_gpio) {
//        if (gpio_cansleep(pw->reset_gpio))
//            gpio_set_value_cansleep(pw->reset_gpio, 1);
//        else
//            gpio_set_value(pw->reset_gpio, 1);
//    }
//
//    /* Need to wait for t4 + t5 + t9 time as per the data sheet */
//    /* t4 - 200us, t5 - 21.2ms, t9 - 1.2ms */
//    usleep_range(23000, 23100);

    pw->state = SWITCH_ON;

    return 0;

//imx273_dvdd_fail:
//    regulator_disable(pw->iovdd);

imx273_iovdd_fail:
    regulator_disable(pw->avdd);

imx273_avdd_fail:
    dev_err(dev, "%s failed.\n", __func__);

    return -ENODEV;
}

/****** imx273_power_off = Power off = 09.2020 **************************/
static int imx273_power_off(struct camera_common_data *s_data)
{

#define TRACE_IMX273_POWER_OFF  0   /* DDD - imx273_power_off() - trace */

    int err = 0;
    struct camera_common_power_rail *pw = s_data->power;
    struct device *dev = s_data->dev;
    struct camera_common_pdata *pdata = s_data->pdata;

#if TRACE_IMX273_POWER_OFF
    dev_err(dev, "%s: power off\n", __func__);
#endif

    if (pdata && pdata->power_off) {
        err = pdata->power_off(pw);
        if (!err) {
            goto power_off_done;
        } else {
            dev_err(dev, "%s failed.\n", __func__);
            return err;
        }
    }

    /* sleeps calls in the sequence below are for internal device
     * signal propagation as specified by sensor vendor
     */
    usleep_range(21, 25);
    if (gpio_is_valid(pw->pwdn_gpio))
        imx273_gpio_set(s_data, pw->pwdn_gpio, 0);
    usleep_range(1, 2);
    if (gpio_is_valid(pw->reset_gpio))
        imx273_gpio_set(s_data, pw->reset_gpio, 0);

    /* datasheet 2.9: reset requires ~2ms settling time*/
    usleep_range(2000, 2010);

    if (pw->iovdd)
        regulator_disable(pw->iovdd);
    if (pw->avdd)
        regulator_disable(pw->avdd);

power_off_done:
    pw->state = SWITCH_OFF;
    return 0;

}

/****** imx273_power_put = Power put = 09.2020 **************************/
static int imx273_power_put(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct camera_common_power_rail *pw = s_data->power;
//    struct camera_common_pdata *pdata = s_data->pdata;
//    struct device *dev = tc_dev->dev;

    if (unlikely(!pw))
        return -EFAULT;


// IMX
    if (likely(pw->dvdd))
        regulator_disable(pw->dvdd);

    if (likely(pw->avdd))
        regulator_put(pw->avdd);

    if (likely(pw->iovdd))
        regulator_put(pw->iovdd);

    pw->dvdd = NULL;
    pw->avdd = NULL;
    pw->iovdd = NULL;

    if (likely(pw->reset_gpio))
        gpio_free(pw->reset_gpio);

    return 0;
}

/****** imx273_power_get = Power get = 09.2020 **************************/
static int imx273_power_get(struct tegracam_device *tc_dev)
{

#define TRACE_IMX273_POWER_GET  0   /* DDD - imx273_power_get() - trace */
#define RESET_GPIO_ENB          0   /* CCC - imx273_power_get() - enable reset_gpio code */

    struct camera_common_data *s_data = tc_dev->s_data;
    struct camera_common_power_rail *pw = s_data->power;
    struct camera_common_pdata *pdata = s_data->pdata;
    struct device *dev = tc_dev->dev;
    struct clk *parent;
//    const char *mclk_name;
//    const char *parentclk_name;
    int err = 0;
//    int ret = 0;

#if TRACE_IMX273_POWER_GET
    dev_info(dev, "%s(): ...\n", __func__);
#endif

    if (!pdata) {
        dev_err(dev, "pdata missing\n");
        return -EFAULT;
    }


// IMX219
    /* Sensor MCLK (aka. INCK) */
    if (pdata->mclk_name) {
        pw->mclk = devm_clk_get(dev, pdata->mclk_name);
        if (IS_ERR(pw->mclk)) {
            dev_err(dev, "unable to get clock %s\n",
                pdata->mclk_name);
            return PTR_ERR(pw->mclk);
        }

        if (pdata->parentclk_name) {
            parent = devm_clk_get(dev, pdata->parentclk_name);
            if (IS_ERR(parent)) {
                dev_err(dev, "unable to get parent clock %s",
                    pdata->parentclk_name);
            } else
                clk_set_parent(pw->mclk, parent);
        }
    }

    /* analog 2.8v */
    if (pdata->regulators.avdd)
    {
        dev_info(dev, "%s: Get regulator avdd\n", __func__);
        err |= camera_common_regulator_get(dev,
                &pw->avdd, pdata->regulators.avdd);
    }
    /* IO 1.8v */
    if (pdata->regulators.iovdd)
    {
        dev_info(dev, "%s: Get regulator iovdd\n", __func__);
        err |= camera_common_regulator_get(dev,
                &pw->iovdd, pdata->regulators.iovdd);
    }
    /* dig 1.2v */
    if (pdata->regulators.dvdd)
    {
        dev_info(dev, "%s: Get regulator dvdd\n", __func__);
        err |= camera_common_regulator_get(dev,
                &pw->dvdd, pdata->regulators.dvdd);
    }
    if (err) {
        dev_err(dev, "%s: unable to get regulator(s)\n", __func__);
        goto done;
    }

    /* Reset or ENABLE GPIO */
#if RESET_GPIO_ENB
    pw->reset_gpio = pdata->reset_gpio;
    if (gpio_is_valid(pw->reset_gpio))
    {
      err = gpio_request(pw->reset_gpio, "cam_reset_gpio");
      if (err < 0) {
          dev_err(dev, "%s: unable to request reset_gpio (%d)\n",
              __func__, err);
          goto done;
      }
    }
#endif

done:
    pw->state = SWITCH_OFF;
#if TRACE_IMX273_POWER_GET
    dev_info(dev, "%s(): err=%d\n", __func__, err);
#endif
    return err;
}

/****** read_property_u32 = Read U32 property = 09.2020 *****************/
static int read_property_u32(
    struct device_node *node, const char *name, int radix, u32 *value)
{
    const char *str;
    int err = 0;
    if(radix != 16) radix = 10;

    err = of_property_read_string(node, name, &str);
    if (err)
        return -ENODATA;

//    err = kstrtou32(str, 10, value);
    err = kstrtou32(str, radix, value);
    if (err)
        return -EFAULT;

    return 0;
}

/****** imx273_parse_dt = Parse DT = 09.2020 ****************************/
static struct camera_common_pdata *imx273_parse_dt(
    struct tegracam_device *tc_dev)
{

#define TRACE_IMX273_PARSE_DT   1   /* DDD - imx273_parse_dt() - trace */

// IMX219 : no parse clocks
//#define PARSE_CLOCKS        0   /* CCC - imx273_parse_dt() - parse clocks */
//#define PARSE_GPIOS         0   /* CCC - imx273_parse_dt() - parse GPIOss */

    struct device *dev = tc_dev->dev;
    struct device_node *node = dev->of_node;
    struct camera_common_pdata *board_priv_pdata;
//    const struct of_device_id *match;
//    struct imx273 *priv = (struct imx273 *)tegracam_get_privdata(tc_dev);

    int gpio;
    int err = 0;
    struct camera_common_pdata *ret = NULL;
    int val = 0;

#if TRACE_IMX273_PARSE_DT
    dev_info(dev, "%s(): ...\n", __func__);
#endif

    if (!node)
        return NULL;

//    match = of_match_device(imx273_of_match, dev);
//    if (!match) {
//        dev_err(dev, "Failed to find matching dt id\n");
//        return NULL;
//    }

    board_priv_pdata = devm_kzalloc(dev,
        sizeof(*board_priv_pdata), GFP_KERNEL);
    if (!board_priv_pdata)
        return NULL;

// do we need reset-gpios ?
    gpio = of_get_named_gpio(node, "reset-gpios", 0);
    if (gpio < 0) {
        if (gpio == -EPROBE_DEFER)
            ret = ERR_PTR(-EPROBE_DEFER);
        dev_err(dev, "reset-gpios not found\n");
//        goto error;
    }
    else
    {
      board_priv_pdata->reset_gpio = (unsigned int)gpio;
    }

// IMX219
    err = of_property_read_string(node, "mclk", &board_priv_pdata->mclk_name);
    if (err)
        dev_err(dev, "%s(): mclk name not present, assume sensor driven externally\n", __func__);

    err = of_property_read_string(node, "avdd-reg",
        &board_priv_pdata->regulators.avdd);
    err |= of_property_read_string(node, "iovdd-reg",
        &board_priv_pdata->regulators.iovdd);
    err |= of_property_read_string(node, "dvdd-reg",
        &board_priv_pdata->regulators.dvdd);
    if (err)
        dev_err(dev, "%s(): avdd, iovdd and/or dvdd reglrs. not present, assume sensor powered independently\n", __func__);

    board_priv_pdata->has_eeprom = of_property_read_bool(node, "has-eeprom");
    board_priv_pdata->v_flip     = of_property_read_bool(node, "vertical-flip");
    board_priv_pdata->h_mirror   = of_property_read_bool(node, "horizontal-mirror");

//............. Read ext. trigger mode from DT
    err = read_property_u32(node, "external-trigger-mode", 10, &val);
    if (err)
    {
        dev_err(dev, "%s(): external-trigger-mode not present in DT, def=%d\n", __func__, ext_trig_mode);
    }
    else
    {
//      priv->sensor_ext_trig = ext_trig_mode;
      ext_trig_mode = val;
      dev_err(dev, "%s(): external-trigger-mode=%d\n", __func__, ext_trig_mode);
    }

//............. Read flash output enable from DT
    err = read_property_u32(node, "flash-output", 10, &val);
    if (err)
    {
        dev_err(dev, "%s(): flash-output not present in DT, def=%d\n", __func__, flash_output);
    }
    else
    {
      flash_output = val;
      dev_err(dev, "%s(): flash-output=%d\n", __func__, flash_output);
    }

//............. Read FPGA address from DT
    err = read_property_u32(node, "fpga_addr", 16, &val);
    if (err)
    {
        dev_err(dev, "%s(): fpga_addr not present in DT, def=%d\n", __func__, fpga_addr);
    }
    else
    {
      fpga_addr = val;
      dev_err(dev, "%s(): fpga_addr=0x%02x\n", __func__, fpga_addr);
    }

//    err = of_property_read_string(node, "num-lanes",  &priv->num_lanes);


#if TRACE_IMX273_PARSE_DT
    dev_err(dev, "%s(): OK\n", __func__);
#endif

    return board_priv_pdata;

//error:
    devm_kfree(dev, board_priv_pdata);
#if TRACE_IMX273_PARSE_DT
    dev_err(dev, "%s(): ERROR\n", __func__);
#endif
    return ret;
}

/****** vc_mipi_reset = Reset VC MIPI sensor = 09.2020 ******************/
// vc_mipi_common_rom_init
static int vc_mipi_reset (
    struct tegracam_device *tc_dev, /* [i/o] tegra camera device        */
    int  sen_mode )                 /* [in] VC sensor mode              */
{

#define TRACE_VC_MIPI_RESET     1   /* DDD - vc_mipi_reset() - trace */

    struct imx273 *priv = (struct imx273 *)tegracam_get_privdata(tc_dev);
    struct device *dev = tc_dev->dev;
    int err = 0;

    static int try=1;
    int addr,reg,data;

    if(!priv->rom)
    {
      dev_err(dev, "%s(): ERROR: VC FPGA not present !!!\n", __func__);
      return -EIO;
    }

    addr = 0x0100; // reset
    data =      2; // powerdown sensor
    reg = reg_write(priv->rom, addr, data);
    if(reg)
      return -EIO;

    if(sen_mode < 0)
    {
        mdelay(200); // wait 200ms

        addr = 0x0101; // status
        reg = reg_read(priv->rom, addr);
        dev_info(dev, "%s: VC_SEN_MODE=%d PowerOFF STATUS=0x%02x\n", __func__, sen_mode, reg);
        return 0;
    }

    addr = 0x0102; // mode
    data = sen_mode; // default 10-bit streaming
    reg = reg_write(priv->rom, addr, data);
    if(reg)
      return -EIO;

    addr = 0x0100; // reset
    data =      0; // powerup sensor
    reg = reg_write(priv->rom, addr, data);
    if(reg)
      return -EIO;

    while(1)
    {
        mdelay(200); // wait 200ms

        addr = 0x0101; // status
        reg = reg_read(priv->rom, addr);

        if(reg & 0x80)
                break;

        if(reg & 0x01)
        {
           dev_err(dev, "%s(): !!! ERROR !!! setting VC Sensor MODE=%d STATUS=0x%02x try=%d\n", __func__, sen_mode,reg,try);
           err = -EIO;
        }

        if(try++ >  4)
            break;
    }

#if TRACE_VC_MIPI_RESET
        dev_info(dev, "%s(): VC_SEN_MODE=%d PowerOn STATUS=0x%02x try=%d\n",__func__, sen_mode,reg, try);
#endif
// ???    mdelay(500);

    try=1; // reset try counter

//done:
#if TRACE_VC_MIPI_RESET
    dev_err(dev, "%s(): sensor_mode=%d err=%d\n", __func__, sen_mode, err);
#endif
    return err;
}

///****** imx273_dump_regs = Dump IMX registers = 09.2020 *****************/
//int imx273_dump_regs(struct imx273 *priv)
//{
//
//#define REG_WRITE_TEST  0   /* DDD - imx273_dump_regs - reg write test */
//
//    struct camera_common_data *s_data = priv->s_data;
//
////    struct camera_common_pdata *pdata = s_data->pdata;
//    struct device *dev = s_data->dev;
////    struct tegracam_device  *tc_dev = priv->tc_dev;
////    u8 reg_val[2];
////    bool eeprom_ctrl = 0;
//    int err = 0;
////    int ret;
//
//    u16 addr;
//    u8  low;
//    u8  mid;
//    u8  high;
//    int sval;
//
//// WINMODE[2:0]  = 0x3007[6:4] : window mode: 0=Full HD 1080p, 1=HD720P, 4=window cropping from full HD 1080p
//    addr = 0x3007;
//#if REG_WRITE_TEST
//    low = 0x40;
//    dev_err(dev, "%s(): IMX273: Write WINMODE = 0x%x\n", __func__, (int) low);
//    imx273_write_reg(s_data, addr, low);
//#endif
//
//    imx273_read_reg(s_data, addr, &low);
//    sval = (low & 0xFF);
//    dev_err(dev, "%s(): IMX273: WINMODE = 0x%x\n", __func__, sval);
//
//
//
//// FRSEL[1:0]      = 0x3009[1:0] : frame rate select: 1=120fps, 2=30fps
//    addr = 0x3009;
//    imx273_read_reg(s_data, addr, &low);
//    sval = (low & 0xFF);
//    dev_err(dev, "%s(): IMX273: FRSEL   = 0x%x\n", __func__, sval);
//
//// WINPH (X0) H,L  = 0x3041,0x3040
//// WINPV (Y0) H,L  = 0x303D,0x303C
//
//// WINWH (DX) H,L  = 0x3043,0x3042
//    addr = 0x3043;
//    imx273_read_reg(s_data, addr, &high);
//    addr = 0x3042;
//    imx273_read_reg(s_data, addr, &low);
//    sval = (high << 8) | (low & 0xFF);
//    dev_err(dev, "%s(): IMX273: sensor width = 0x%x (%d)\n", __func__, sval,sval);
//
//
//
//// WINWV (DY) H,L  = 0x303F,0x303E
//    addr = 0x303F;
//    imx273_read_reg(s_data, addr, &high);
//    addr = 0x303E;
//    imx273_read_reg(s_data, addr, &low);
//    sval = (high << 8) | (low & 0xFF);
//    dev_err(dev, "%s(): IMX273: sensor height = 0x%x (%d)\n", __func__, sval,sval);
//
//// WINWV_OB[3:0]   = 0x303A
//    addr = 0x303A;
//    imx273_read_reg(s_data, addr, &low);
//    sval = (low & 0xFF);
//    dev_err(dev, "%s(): IMX273: WINWV_OB = 0x%x (%d)\n", __func__, sval,sval);
//
//// X_OUT_SIZE[12:0] H,L = 0x3473,0x3472 = Horizontal (H) direction effective pixel width setting.
//    addr = 0x3473;
//    imx273_read_reg(s_data, addr, &high);
//    addr = 0x3472;
//    imx273_read_reg(s_data, addr, &low);
//    sval = (high << 8) | (low & 0xFF);
//    dev_err(dev, "%s(): IMX273: X_OUT_SIZE = 0x%x (%d)\n", __func__, sval,sval);
//
//// Y_OUT_SIZE H,L [12:0] = 0x3419,0x3418 = Vertical direction effective pixels
//    addr = 0x3419;
//    imx273_read_reg(s_data, addr, &high);
//    addr = 0x3418;
//    imx273_read_reg(s_data, addr, &low);
//    sval = (high << 8) | (low & 0xFF);
//    dev_err(dev, "%s(): IMX273: Y_OUT_SIZE = 0x%x (%d)\n", __func__, sval,sval);
//
//// HMAX[15:0] H,L =  0x301D,0x301C
//    addr = 0x301D;
//    imx273_read_reg(s_data, addr, &high);
//    addr = 0x301C;
//    imx273_read_reg(s_data, addr, &low);
//    sval = (high << 8) | (low & 0xFF);
//    dev_err(dev, "%s(): IMX273: HMAX = 0x%x (%d)\n", __func__, sval,sval);
//
//// VMAX[17:0] H,M,L =  0x301A,0x3019,0x3018
//    addr = 0x301A;
//    imx273_read_reg(s_data, addr, &high);
//    addr = 0x3019;
//    imx273_read_reg(s_data, addr, &mid);
//    addr = 0x3018;
//    imx273_read_reg(s_data, addr, &low);
//    sval = (high << 16) | (mid << 8) | (low & 0xFF);
//    dev_err(dev, "%s(): IMX273: VMAX = 0x%x (%d)\n", __func__, sval,sval);
//
////// embedded data line control register
////    addr = 0xBCF1;
//////    low = 2;
//////    imx273_write_reg(s_data, addr, low);
//////    dev_err(dev, "%s(): IMX412: Set: embedded data line control = 2\n", __func__);
////    imx273_read_reg(s_data, addr, &high);
////    dev_err(dev, "%s(): IMX273: embedded data line control = 0x%x\n", __func__, (int)high);
//
////done:
//    return err;
//}

/****** vc_mipi_common_trigmode_write = Trigger mode setup = 10.2020 ****/
static int vc_mipi_common_trigmode_write(struct i2c_client *rom, u32 sensor_ext_trig, u32 exposure_time, u32 io_config, u32 enable_extrig, u32 sen_clk)
{
    int ret;

    if(sensor_ext_trig)
    {
        u64 exposure = (exposure_time * (sen_clk/1000000)); // sen_clk default=54Mhz imx183=72Mhz

        //exposure = (exposure_time * 24 or 25?); //TODO OV9281
        //exposure = (exposure_time * 54); //default
        //exposure = (exposure_time * 72); //TEST IMX183
        //printk("ext_trig exposure=%lld",exposure);

        int addr = 0x0108; // ext trig enable
        //int data =      1; // external trigger enable
        //int data =      2; // external static trigger variable shutter enable
        //int data =      4; // TEST external self trigger enable
        int data = enable_extrig; // external trigger enable
        ret = reg_write(rom, addr, data);

        addr = 0x0103; // io configuration
        data = io_config;
        ret |= reg_write(rom, addr, data);

        addr = 0x0109; // shutter lsb
        data = exposure & 0xff;
        ret |= reg_write(rom, addr, data);

        addr = 0x010a;
        data = (exposure >> 8) & 0xff;
        ret |= reg_write(rom, addr, data);

        addr = 0x010b;
        data = (exposure >> 16) & 0xff;
        ret |= reg_write(rom, addr, data);

        addr = 0x010c; // shutter msb
        data = (exposure >> 24) & 0xff;
        ret |= reg_write(rom, addr, data);

    }
    else
    {
        int addr = 0x0108; // ext trig enable
        int data =      0; // external trigger disable
        ret = reg_write(rom, addr, data);

        addr = 0x0103; // io configuration
        data = io_config;
        ret |= reg_write(rom, addr, data);
    }
    return ret;
}

/****** imx273_start_streaming = Start streaming = 09.2020 **************/
static int imx273_start_streaming(struct tegracam_device *tc_dev)
{

#define TRACE_IMX273_START_STREAMING       1   /* DDD - imx273_start_streaming() - trace */
#define IMX273_START_STREAMING_SET_CTRLS   1   /* CCC - imx273_start_streaming() - set controls */
#define IMX273_START_STREAMING_DELAY       0   // 300 /* CCC - imx273_start_streaming() - delay in ms after start, 0=off */
#define VC_EXT_TRIG_SET_EXP                1   /* CCC - imx273_start_streaming() - set exposure in ext. trigger code */

    struct imx273 *priv = (struct imx273 *)tegracam_get_privdata(tc_dev);
//    struct imx273 *priv = (struct imx273 *)tc_dev->priv;
    struct device *dev = tc_dev->dev;
    int err = 0;

#if TRACE_IMX273_START_STREAMING
//    dev_err(dev, "%s():...\n", __func__);
#endif


// Set gain and exposure before streaming start
#if IMX273_START_STREAMING_SET_CTRLS
      imx273_set_gain(tc_dev, priv->digital_gain);
      imx273_set_exposure(tc_dev, priv->exposure_time);
      imx273_set_frame_rate(tc_dev, priv->frame_rate);
//      imx273_set_frame_length(tc_dev, priv->frame_length);
      mdelay(100);
#endif

#if TRACE_IMX273_START_STREAMING
//    dev_info(dev, "%s(): Set frame_length=%d\n", __func__, IMX273_DEFAULT_FRAME_LENGTH);
//    dev_info(dev, "%s(): Set gain=%d\n", __func__, IMX273_DEFAULT_GAIN);
#endif

//............... Set trigger mode: on/off
#if VC_CODE // [[[
{
    int ret = 0;

    if(priv->sensor_ext_trig)
    {
//        u64 exposure = (priv->exposure_time * 10000) / 185185;
        u64 exposure = (priv->exposure_time * (priv->sen_clk/1000000)); // sen_clk default=54Mhz imx183=72Mhz
        int addr;
        int data;

#if TRACE_IMX273_START_STREAMING
       dev_err(dev, "%s(): sensor_ext_trig=%d, exposure=%llu (0x%llx)\n", __func__, priv->sensor_ext_trig, exposure, exposure);
#endif

        addr = 0x0108; // ext trig enable
        // data =      1; // external trigger enable
        // data =      4; // TEST external self trigger enable
        data = priv->sensor_ext_trig; // external trigger enable
        ret += reg_write(priv->rom, addr, data);

        addr = 0x0103; // flash output enable
//        data =      1; // flash output enable
        data = priv->flash_output; // flash output enable
        ret += reg_write(priv->rom, addr, data);

#if VC_EXT_TRIG_SET_EXP    /* [[[ */
        addr = 0x0109; // shutter lsb
        data = exposure & 0xff;
        ret += reg_write(priv->rom, addr, data);

        addr = 0x010a;
        data = (exposure >> 8) & 0xff;
        ret += reg_write(priv->rom, addr, data);

        addr = 0x010b;
        data = (exposure >> 16) & 0xff;
        ret += reg_write(priv->rom, addr, data);

        addr = 0x010c; // shutter msb
        data = (exposure >> 24) & 0xff;
        ret += reg_write(priv->rom, addr, data);
#endif  /* ]]] */

    }
    else
    {
        int addr = 0x0108; // ext trig enable
        int data =      0; // external trigger disable
        ret = reg_write(priv->rom, addr, data);

        addr = 0x0103; // flash output enable
        data = priv->flash_output; // flash output enable
        ret += reg_write(priv->rom, addr, data);

        if(ret)
        {
          dev_err(dev, "%s(): reg_write() error=%d\n", __func__, ret);
          goto exit;
        }

//        return reg_write_table(client, start);
    }
    mdelay(10);    //  ms

    if(ret)
    {
      dev_err(dev, "%s(): reg_write() error=%d\n", __func__, ret);
      goto exit;
    }
}
#endif  // ]]]

//............... Start streaming
//    mutex_lock(&priv->streaming_lock);
    err = imx273_write_table(priv, imx273_mode_table[IMX273_MODE_START_STREAM]);
    if (err) {
//        mutex_unlock(&priv->streaming_lock);
        goto exit;
    } else {
        priv->streaming = true;
//        mutex_unlock(&priv->streaming_lock);
    }

    usleep_range(50000, 51000);
//    mdelay(300);    // ??? delay after start streaming

#if IMX273_START_STREAMING_DELAY
    dev_err(dev, "%s(): Delay after streaming start: %d ms\n", __func__, IMX273_START_STREAMING_DELAY);
    mdelay(IMX273_START_STREAMING_DELAY);    // ??? delay after start streaming
#endif

exit:
#if TRACE_IMX273_START_STREAMING
    dev_err(dev, "%s(): err=%d\n", __func__, err);
#endif

    return err;

}

/****** imx273_stop_streaming = Stop streaming = 09.2020 ****************/
static int imx273_stop_streaming(struct tegracam_device *tc_dev)
{

#define TRACE_IMX273_STOP_STREAMING     1   /* DDD - imx273_stop_streaming() - trace */
//#define IMX273_STOP_STREAMING_ENB_RESET    0    /* CCC - imx273_stop_streaming() - enable VC MIPI reset */

//    struct camera_common_data *s_data = tc_dev->s_data;
    struct imx273 *priv = (struct imx273 *)tegracam_get_privdata(tc_dev);
//    struct imx273 *priv = (struct imx273 *)tc_dev->priv;
    struct device *dev = tc_dev->dev;
    int err = 0;

//    mutex_lock(&priv->mutex);

#if STOP_STREAMING_SENSOR_RESET
{
// VC RPI driver:
// /* reinit sensor */
//    ret = vc_mipi_common_rom_init(client, priv->rom, -1);
    err = vc_mipi_reset (tc_dev, -1);
    if (err)
      return err;

//    ret = vc_mipi_common_rom_init(client, priv->rom, priv->cur_mode->sensor_mode);
    err = vc_mipi_reset (tc_dev, sensor_mode);
    if (err)
      return err;

    err = vc_mipi_common_trigmode_write(priv->rom, 0, 0, 0, 0, 0); /* disable external trigger counter */
    if (err)
        dev_err(dev, "%s: REINIT: Error %d disabling trigger counter\n", __func__, err);
//
//              ret = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
//              if (ret < 0)
//                  dev_err(&client->dev, "REINIT: Error %d setting controls\n", ret);
//                return vc_mipi_common_reg_write_table(client, priv->trait->stop);
//                break;
}
#endif

#if TRACE_IMX273_STOP_STREAMING
//    dev_err(dev, "%s():\n", __func__);
#endif

    err = imx273_write_table(priv, imx273_mode_table[IMX273_MODE_STOP_STREAM]);
    if (err)
    {
//#if TRACE_IMX273_STOP_STREAMING
      dev_err(dev, "%s(): imx273_write_table() err=%d\n", __func__, err);
//#endif
//        mutex_unlock(&priv->streaming_lock);
        goto exit;
    }
    else
    {
        priv->streaming = false;

//#if IMX273_STOP_STREAMING_ENB_RESET  // [[[
//        err = vc_mipi_reset(tc_dev, sensor_mode);
//        if(err)
//        {
//          dev_err(dev, "%s(): VC MIPI sensor reset: err=%d\n", __func__, err);
//        }
//        else
//        {
//          dev_err(dev, "%s(): VC sensor reset\n", __func__);
//        }
//
//#endif // ]]] - VC reset
    }

//    usleep_range(10000, 11000);
    usleep_range(50000, 51000);

exit:
#if TRACE_IMX273_STOP_STREAMING
    dev_err(dev, "%s(): err=%d\n\n", __func__, err);
#endif

//    mutex_unlock(&priv->mutex);
    return err;

}

static int imx273_set_mode(struct tegracam_device *tc_dev);

static struct camera_common_sensor_ops imx273_common_ops = {
    .numfrmfmts = ARRAY_SIZE(imx273_frmfmt),
    .frmfmt_table = imx273_frmfmt,
    .power_on = imx273_power_on,
    .power_off = imx273_power_off,
    .write_reg = imx273_write_reg,
    .read_reg = imx273_read_reg,
    .parse_dt = imx273_parse_dt,
    .power_get = imx273_power_get,
    .power_put = imx273_power_put,
    .set_mode = imx273_set_mode,
    .start_streaming = imx273_start_streaming,
    .stop_streaming = imx273_stop_streaming,
};

/****** imx273_set_mode = Set mode = 09.2020 ****************************/
static int imx273_set_mode(struct tegracam_device *tc_dev)
{

#define TRACE_IMX273_SET_MODE            1   /* DDD - imx273_set_mode() - trace */
#define IMX273_SET_MODE_DUMP_DT_PARAMS   0   /* DDD - imx273_set_mode() - dump DT params */
#define DUMP_SENSOR_MODE                 1   /* DDD - imx273_set_mode() - dump sensor mode */
#define DUMP_IMX273_REGS1                0   /* DDD - imx273_set_mode() - dump sensor regs aft set mode */

//#if !SKIP_IMX273_SET_MODE
    struct imx273 *priv = (struct imx273 *)tegracam_get_privdata(tc_dev);
//#endif
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = tc_dev->dev;
    const struct camera_common_colorfmt *colorfmt = s_data->colorfmt;
    int pix_fmt = colorfmt->pix_fmt;
    int err = 0;
    int sensor_mode_id;

#if IMX273_TRIG_FIX || IMX273_SET_MODE_DUMP_DT_PARAMS
    struct sensor_mode_properties *mode =  &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
#endif

#if DUMP_SENSOR_MODE
    dev_err(dev, "%s: sensor_mode_id=%d use_sensor_mode_id=%d\n", __func__, s_data->sensor_mode_id, s_data->use_sensor_mode_id);
#endif

/*............. Set new sensor mode */
// IMX273 sensor modes:
//   0x00 :  8bit, 2 lanes, streaming           V4L2_PIX_FMT_GREY 'GREY', V4L2_PIX_FMT_SRGGB8  'RGGB'
//   0x01 : 10bit, 2 lanes, streaming           V4L2_PIX_FMT_Y10  'Y10 ', V4L2_PIX_FMT_SRGGB10 'RG10'
//   0x02 : 12bit, 2 lanes, streaming           V4L2_PIX_FMT_Y12  'Y12 ', V4L2_PIX_FMT_SRGGB12 'RG12'
//   0x03 :  8bit, 2 lanes, external trigger
//   0x04 : 10bit, 2 lanes, external trigger
//   0x05 : 12bit, 2 lanes, external trigger
//
//   0x06 :  8bit, 4 lanes, streaming
//   0x07 : 10bit, 4 lanes, streaming
//   0x08 : 12bit, 4 lanes, streaming
//   0x09 :  8bit, 4 lanes, external trigger global shutter reset
//   0x0A : 10bit, 4 lanes, external trigger global shutter reset
//   0x0B : 12bit, 4 lanes, external trigger global shutter reset

    if(pix_fmt == V4L2_PIX_FMT_GREY || pix_fmt == V4L2_PIX_FMT_SRGGB8)
    {
      sensor_mode = 0;      // 8-bit
    }
    else if(pix_fmt == V4L2_PIX_FMT_Y10 || pix_fmt == V4L2_PIX_FMT_SRGGB10)
    {
      sensor_mode = 1;      // 10-bit
    }
    else if(pix_fmt == V4L2_PIX_FMT_Y12 || pix_fmt == V4L2_PIX_FMT_SRGGB12)
    {
      sensor_mode = 2;      // 12-bit
    }

    if(priv->num_lanes == 4)
    {
      sensor_mode += 6;
    }


/*----------------------------------------------------------------------*/
/*                  Static ext. trigger from DT                         */
/*----------------------------------------------------------------------*/
    if(ext_trig_mode >= 0)
    {
      if(ext_trig_mode > 0)
      {
        sensor_mode += 3;
      }
      priv->sensor_ext_trig = ext_trig_mode;   // 0=trig off, 1=trig on, 4=trig test
#if TRACE_IMX273_SET_MODE
      dev_err(dev, "%s(): Ext. trig from DT: New sensor_mode=%d (0-2=8,10,12bit, 3-5=8,10,12bit trig), sensor_ext_trig=%d\n", __func__,
                         sensor_mode, priv->sensor_ext_trig);
#endif
    }

/*----------------------------------------------------------------------*/
/*             Dynamic ext. trigger from sensor_mode CTL                */
/*----------------------------------------------------------------------*/
    else
    {

#if TRACE_IMX273_SET_MODE
      dev_err(dev, "%s(): Dynamic ext. trig...\n", __func__);
#endif

// Get sensor mode:
#if IMX273_TRIG_MODE
      sensor_mode_id = 1;     // force ext.trigger mode
      dev_err(dev, "%s: Force ext. trigger mode !!!!\n", __func__);
#else
      sensor_mode_id = s_data->sensor_mode_id;
#endif

//    if(sensor_mode_id == 0 || sensor_mode_id == 1 || sensor_mode_id == 2)      // 0=free run, 1=ext. trigger, 2=trigger self test
//    {
//      sensor_mode = sensor_mode_id ? 4 : 1;
//    }

      if(sensor_mode_id)      // 0=free run, 1=ext. trigger, 2=trigger self test
      {
        sensor_mode += 3;

#if IMX273_TRIG_FIX   // trig mode fix
        tc_dev->s_data->numfmts = ARRAY_SIZE(imx273_trig_frmfmt);  // tc_dev->sensor_ops->numfrmfmts;
        tc_dev->s_data->frmfmt  = imx273_trig_frmfmt;             // tc_dev->sensor_ops->frmfmt_table;

        imx273_common_ops.numfrmfmts = ARRAY_SIZE(imx273_trig_frmfmt);
        imx273_common_ops.frmfmt_table = imx273_trig_frmfmt;

        mode->image_properties.height = IMX273_DY-2;
        s_data->fmt_height = IMX273_DY-2;
#endif
      }
#if IMX273_TRIG_FIX   // trig mode fix
      else    // free-run mode
      {
        tc_dev->s_data->numfmts = ARRAY_SIZE(imx273_frmfmt);  // tc_dev->sensor_ops->numfrmfmts;
        tc_dev->s_data->frmfmt  = imx273_frmfmt;              // tc_dev->sensor_ops->frmfmt_table;

        imx273_common_ops.numfrmfmts = ARRAY_SIZE(imx273_frmfmt);
        imx273_common_ops.frmfmt_table = imx273_frmfmt;

        mode->image_properties.height = IMX273_DY;
        s_data->fmt_height = IMX273_DY;
      }
#endif
      if(sensor_mode_id == 0)
        priv->sensor_ext_trig = 0;      // 0=trig off, 1=trig on, 4=trig test
      else if(sensor_mode_id == 1)
        priv->sensor_ext_trig = 1;      // 0=trig off, 1=trig on, 4=trig test
      else if(sensor_mode_id == 2)
        priv->sensor_ext_trig = 4;      // 0=trig off, 1=trig on, 4=trig test
    } /* else: dynamic ext. trigger */


/*----------------------------------------------------------------------*/
/*                       Change VC MIPI sensor mode                     */
/*----------------------------------------------------------------------*/
    if(priv->sensor_mode != sensor_mode)
    {
      priv->sensor_mode = sensor_mode;


#if TRACE_IMX273_SET_MODE
      dev_err(dev, "%s(): New sensor_mode=%d (0-2=8,10,12bit, 3-5=8,10,12bit trig, 6-11=4-lanes), sensor_ext_trig=%d\n", __func__,
                         sensor_mode, priv->sensor_ext_trig);
#endif

      err = vc_mipi_reset(tc_dev, sensor_mode);
// ??? Xavier: switch between 8-bit and 10-bit modes
//      mdelay(300); // wait 300ms : added in vc_mipi_reset()
      if(err)
      {
          dev_err(dev, "%s(): vc_mipi_set_mode() error=%d\n", __func__, err);
      }

    }


// Get camera mode:
//    if(s_data->fmt_width >= 1920 && s_data->fmt_height >= 1080)
    {
      priv->cam_mode = IMX273_MODE_1440X1080;
    }
//    else
//    {
//      priv->cam_mode = IMX297_MODE_720X540;
//    }


//#if TRACE_IMX273_SET_MODE
//    dev_err(dev, "%s(): cam_mode=%d\n", __func__, priv->cam_mode);
//#endif


// Set camera mode:
// Note: After each streaming stop, the sensor is initialized to default mode:
//       by vc_mipi_reset(), but this is not our default mode
//    if(priv->cam_mode != IMX273_MODE_1440X1080)
    {
      err = imx273_write_table(priv, imx273_mode_table[priv->cam_mode]);
      if (err)
      {
        dev_err(dev, "%s(): imx273_write_table() error=%d\n", __func__, err);
//        goto exit;
      }
      else
      {
#if TRACE_IMX273_SET_MODE
//        dev_err(dev, "%s(): imx273_write_table() OK, cam_mode=%d\n", __func__, priv->cam_mode);
#endif
      }
    }

#if IMX273_SET_MODE_DUMP_DT_PARAMS
{
//    struct sensor_mode_properties *mode =  &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
    int mclk_freq      = (int)mode->signal_properties.mclk_freq;
    int pixel_clock    = (int)mode->signal_properties.pixel_clock.val;
    int cil_settletime = (int)mode->signal_properties.cil_settletime;
    int discontinuous_clk = (int)mode->signal_properties.discontinuous_clk;

//struct sensor_image_properties {
//    __u32 width;
//    __u32 height;
//    __u32 line_length;
//    __u32 pixel_format;
//    __u32 embedded_metadata_height;
//};
//
//struct camera_common_data {
//    struct camera_common_sensor_ops     *ops;
//    struct v4l2_ctrl_handler            *ctrl_handler;
//    struct device                       *dev;
//    const struct camera_common_frmfmt   *frmfmt;
//    const struct camera_common_colorfmt *colorfmt;
//    struct dentry                       *debugdir;
//    struct camera_common_power_rail     *power;
//
//    struct v4l2_subdev          subdev;
//    struct v4l2_ctrl            **ctrls;
//
//    struct sensor_properties        sensor_props;
//
//    /* TODO: cleanup neeeded once all the sensors adapt new framework */
//    struct tegracam_ctrl_handler    *tegracam_ctrl_hdl;
//    struct regmap                   *regmap;
//    struct camera_common_pdata      *pdata;
//
//    /* TODO: cleanup needed for priv once all the sensors adapt new framework */
//    void    *priv;
//    int numctrls;
//    int csi_port;
//    int numlanes;   // bus-width
//    int mode;
//    int mode_prop_idx;
//    int numfmts;
//    int def_mode, def_width, def_height;
//    int def_clk_freq;
//    int fmt_width, fmt_height;
//    int sensor_mode_id;
//    bool    use_sensor_mode_id;
//    bool    override_enable;
//    u32 version;
//};

    dev_err(dev, "%s: mode=%d mode_prop_idx=%d\n", __func__, s_data->mode, s_data->mode_prop_idx);
//    dev_err(dev, "%s: sensor_mode_id=%d use_sensor_mode_id=%d\n", __func__, s_data->sensor_mode_id, s_data->use_sensor_mode_id);
    dev_err(dev, "%s(): mclk_freq=%d pixel_clock=%d cil_settletime=%d discontinuous_clk=%d\n", __func__,
         mclk_freq, pixel_clock, cil_settletime, discontinuous_clk);
    pix_fmt = (int)mode->image_properties.pixel_format;
    dev_err(dev, "%s(): width,height,line_length=%d,%d,%d pix_fmt=0x%x '%c%c%c%c' embedded_metadata_height=%d\n", __func__,
         (int)mode->image_properties.width,
         (int)mode->image_properties.height,
         (int)mode->image_properties.line_length,
//         (int)mode->image_properties.pixel_format,
                        pix_fmt,
                        (char)((pix_fmt      ) & 0xFF),
                        (char)((pix_fmt >>  8) & 0xFF),
                        (char)((pix_fmt >> 16) & 0xFF),
                        (char)((pix_fmt >> 24) & 0xFF),
         (int)mode->image_properties.embedded_metadata_height);

//    imx273_dump_regs(priv);

}
#endif


#if TRACE_IMX273_SET_MODE
{
    dev_err(dev, "%s(): fmt_width,fmt_height=%d,%d pix_fmt=0x%x '%c%c%c%c', cam_mode=%d, err=%d\n", __func__,
                        s_data->fmt_width, s_data->fmt_height,
                        pix_fmt,
                        (char)((pix_fmt      ) & 0xFF),
                        (char)((pix_fmt >>  8) & 0xFF),
                        (char)((pix_fmt >> 16) & 0xFF),
                        (char)((pix_fmt >> 24) & 0xFF),
                        priv->cam_mode, err);

}
#endif


//      mdelay(300);

#if DUMP_IMX273_REGS1
{
    dev_err(dev, "%s(): Dump sensor regs\n", __func__);
    imx273_dump_regs(priv);
}
#endif

    return err;
}

/****** imx273_video_probe = Video probe = 09.2020 **********************/
static int imx273_video_probe(struct i2c_client *client)
{

    int ret = 0;

#if 0
    u16 model_id;
    u32 lot_id=0;
    u16 chip_id=0;

    /* Check and show model, lot, and chip ID. */
    ret = reg_read(client, 0x359b);
    if (ret < 0) {
        dev_err(&client->dev, "Failure to read Model ID (high byte)\n");
        goto done;
    }
    model_id = ret << 8;

    ret = reg_read(client, 0x359a);
    if (ret < 0) {
        dev_err(&client->dev, "Failure to read Model ID (low byte)\n");
        goto done;
    }
    model_id |= ret;
#if 0
    if ( ! ((model_id == 296) || (model_id == 202)) ) {
        dev_err(&client->dev, "Model ID: %x not supported!\n",
            model_id);
        ret = -ENODEV;
        goto done;
    }
#endif
    dev_err(&client->dev,
         "Model ID 0x%04x, Lot ID 0x%06x, Chip ID 0x%04x\n",
         model_id, lot_id, chip_id);
done:
#endif

    return ret;
}

// new video probe:
///****** imx_video_probe = IMX video probe = 09.2020 *********************/
//// Check and compare model
//static int imx_video_probe(struct i2c_client *client, struct imx *priv)
//{
//    u16 model_id, check_id;
//    u32 lot_id=0;
//    u16 chip_id=0;
//    int ret;
//
//    /* Check and show model, lot, and chip ID. */
//    ret = vc_mipi_common_reg_read(client, sen_reg(priv, MODEL_ID_HIGH));
//    if (ret < 0) {
//        dev_err(&client->dev, "Failure to read Model ID (high byte)\n");
//        goto done;
//    }
//    model_id = (ret << 8) & 0xff00;
//
//    ret = vc_mipi_common_reg_read(client, sen_reg(priv, MODEL_ID_LOW));
//    if (ret < 0) {
//        dev_err(&client->dev, "Failure to read Model ID (low byte)\n");
//        goto done;
//    }
//    model_id |= ret & 0x00ff;
//
//
//    switch(priv->model)
//    {
//        case IMX290_MODEL_MONOCHROME:
//        case IMX327_MODEL_MONOCHROME:
//        case IMX290_MODEL_COLOR:
//        case IMX327_MODEL_COLOR:
//            check_id = 0x07D0;
//            break;
//        case IMX273_MODEL_MONOCHROME:
//        case IMX273_MODEL_COLOR:
//        case IMX296_MODEL_MONOCHROME:
//        case IMX297_MODEL_MONOCHROME:
//        case IMX296_MODEL_COLOR:
//        case IMX297_MODEL_COLOR:
//            check_id = 0x0494;
//            break;
//        case IMX412_MODEL_MONOCHROME:
//        case IMX412_MODEL_COLOR:
//            check_id = 0x0577;
//            break;
//        case OV9281_MODEL_MONOCHROME:
//        case OV9281_MODEL_COLOR:
//            check_id = 0x9281;
//            break;
//        case IMX183_MODEL_MONOCHROME:
//        case IMX183_MODEL_COLOR:
//            check_id = 0x0183;
//            break;
//        default:
//            return -EINVAL;
//    }
//
//    if ( ! ((model_id == check_id)) ) {
//        dev_err(&client->dev, "Model ID: %x not supported!\n",
//                model_id);
//        ret = -ENODEV;
//        goto done;
//    }
//
//
//    if( sen_reg(priv, CHIP_REV) ) // chip_id register not always availabale
//    {
//        ret = vc_mipi_common_reg_read(client, sen_reg(priv, CHIP_REV));
//        if (ret < 0) {
//            dev_err(&client->dev, "Failure to read Chip ID (low byte)\n");
//            goto done;
//        }
//        chip_id = ret;
//    }
//
//    dev_info(&client->dev,
//            "Model ID 0x%04x, Lot ID 0x%06x, Chip ID 0x%04x\n",
//            model_id, lot_id, chip_id);
//done:
//    return ret;
//}


/****** imx273_probe_vc_rom = Probe VC ROM = 09.2020 ********************/
static struct i2c_client * imx273_probe_vc_rom(struct i2c_adapter *adapter, u8 addr)
{
    struct i2c_board_info info = {
        I2C_BOARD_INFO("dummy", addr),
    };
    unsigned short addr_list[2] = { addr, I2C_CLIENT_END };

    return i2c_new_probed_device(adapter, &info, addr_list, NULL);
}


/****** imx273_board_setup = Board setup = 09.2020 **********************/
static int imx273_board_setup(struct imx273 *priv)
{

#define TRACE_IMX273_BOARD_SETUP   1   /* DDD - imx273_board_setup() - trace */
#define DUMP_CTL_REG_DATA          0   /* DDD - imx273_board_setup() - dump module control registers 0x100-0x108 (I2C addr=0x10) */
#define DUMP_HWD_DESC_ROM_DATA     0   /* DDD - imx273_board_setup() - dump Hardware Desriptor ROM data (I2C addr=0x10) */
#define DUMP_IMX273_REGS           0   /* DDD - imx273_board_setup() - dump IMX273 regs */
#define DUMP_V4L_PARAMS            1   /* DDD - imx273_board_setup() - dump V4L params */
#define DUMP_ROM_TABLE_REGS        0   /* DDD - imx273_board_setup() - dump ROM table regs */

//#define MOD_ID                0x0273   /* CCC - imx273_board_setup() - module id., 0=off: don't check */
//#define ERR_MOD_ID                 0   /* DDD - imx273_board_setup() - invalid module id test */

    struct camera_common_data *s_data = priv->s_data;
    struct camera_common_pdata *pdata = s_data->pdata;
    struct device *dev = s_data->dev;
    struct tegracam_device  *tc_dev = priv->tc_dev;
//    u8 reg_val[2];
//    bool eeprom_ctrl = 0;
    int err = 0;
    int ret;

#if VC_CODE    // [[[ - new VC code
    struct i2c_client *client = priv->i2c_client;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct sensor_mode_properties *mode =  &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
//    int num_lanes;

#if DUMP_V4L_PARAMS
    int mclk_freq      = (int)mode->signal_properties.mclk_freq;
    int pixel_clock    = (int)mode->signal_properties.pixel_clock.val;
    int cil_settletime = (int)mode->signal_properties.cil_settletime;
    int discontinuous_clk = (int)mode->signal_properties.discontinuous_clk;
    int pix_fmt           = (int)mode->image_properties.pixel_format;


//struct sensor_image_properties {
//    __u32 width;
//    __u32 height;
//    __u32 line_length;
//    __u32 pixel_format;
//    __u32 embedded_metadata_height;
//};

    dev_err(dev, "%s: mclk_freq=%d pixel_clock=%d cil_settletime=%d discontinuous_clk=%d\n", __func__,
                        mclk_freq, pixel_clock, cil_settletime, discontinuous_clk);
    dev_err(dev, "%s: width,height,line_length=%d,%d,%d pix_fmt=0x%x '%c%c%c%c' embedded_metadata_height=%d\n", __func__,
                        (int)mode->image_properties.width,
                        (int)mode->image_properties.height,
                        (int)mode->image_properties.line_length,
                        pix_fmt,
                        (char)((pix_fmt      ) & 0xFF),
                        (char)((pix_fmt >>  8) & 0xFF),
                        (char)((pix_fmt >> 16) & 0xFF),
                        (char)((pix_fmt >> 24) & 0xFF),
                        (int)mode->image_properties.embedded_metadata_height);
#endif

#endif  // ]]] - end of VC_CODE

    if (pdata->mclk_name) {
        err = camera_common_mclk_enable(s_data);
        if (err) {
            dev_err(dev, "%s: error turning on mclk (%d)\n", __func__, err);
            goto done;
        }
    }

    err = imx273_power_on(s_data);
    if (err) {
        dev_err(dev, "%s: error during power on sensor (%d)\n", __func__, err);
        goto err_power_on;
    }

/*----------------------------------------------------------------------*/
#if VC_CODE    // [[[ - VC code
/*----------------------------------------------------------------------*/

    priv->num_lanes = (int)mode->signal_properties.num_lanes;
    if(priv->num_lanes == 2)
    {
        sensor_mode = 1;   // autoswitch to sensor_mode=0 if 2 lanes are given
    }
    else if(priv->num_lanes == 4)
    {
        sensor_mode = 7;    // autoswitch to sensor_mode=1 if 4 lanes are given
    }
    else
    {
        dev_err(dev, "%s: VC Sensor device-tree: Invalid number of data-lanes: %d\n",__func__, priv->num_lanes);
        return -EINVAL;
    }
    dev_err(dev, "%s: VC Sensor device-tree has configured %d data-lanes: sensor_mode=%d\n",__func__, priv->num_lanes, sensor_mode);

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
        dev_err(&client->dev, "%s(): I2C-Adapter doesn't support I2C_FUNC_SMBUS_BYTE\n", __func__);
        return -EIO;
    }

//    priv->rom = i2c_new_dummy(adapter,fpga_addr);
    priv->rom = imx273_probe_vc_rom(adapter,priv->fpga_addr);

    if ( priv->rom )
    {
//        static int i=1;
        int addr,reg;

#if DUMP_HWD_DESC_ROM_DATA      /* dump Hardware Desriptor ROM data */
        dev_err(&client->dev, "%s(): Dump Hardware Descriptor ROM data:\n", __func__);
#endif

        for (addr=0; addr<sizeof(priv->rom_table); addr++)
        {
          reg = reg_read(priv->rom, addr+0x1000);
          if(reg < 0)
          {
              i2c_unregister_device(priv->rom);
              return -EIO;
          }
          *((char *)(&(priv->rom_table))+addr)=(char)reg;
#if DUMP_HWD_DESC_ROM_DATA      /* [[[ - dump Hardware Desriptor ROM data */
{
          static int sval = 0;   // short 2-byte value

          if(addr & 1)  // odd addr
          {
            sval |= (int)reg << 8;
            dev_err(&client->dev, "addr=0x%04x reg=0x%04x\n",addr+0x1000-1,sval);
          }
          else
          {
            sval = reg;
          }
}
#endif  // ]]]

        } /* for (addr=0; addr<sizeof(priv->rom_table); addr++) */

        dev_err(&client->dev, "%s(): VC FPGA found!\n", __func__);

        dev_err(&client->dev, "[ MAGIC  ] [ %s ]\n",
                priv->rom_table.magic);

        dev_err(&client->dev, "[ MANUF. ] [ %s ] [ MID=0x%04x ]\n",
                priv->rom_table.manuf,
                priv->rom_table.manuf_id);

        dev_err(&client->dev, "[ SENSOR ] [ %s %s ]\n",
                priv->rom_table.sen_manuf,
                priv->rom_table.sen_type);

        dev_err(&client->dev, "[ MODULE ] [ ID=0x%04x ] [ REV=0x%04x ]\n",
                priv->rom_table.mod_id,
                priv->rom_table.mod_rev);

        dev_err(&client->dev, "[ MODES  ] [ NR=0x%04x ] [ BPM=0x%04x ]\n",
                priv->rom_table.nr_modes,
                priv->rom_table.bytes_per_mode);

//................ Check model:
        priv->model = IMX_MODEL_NONE;
        if(priv->rom_table.sen_type)
        {
            u32 len = strnlen(priv->rom_table.sen_type,16);
            if(len > 0 && len < 17)
            {
                // IMX273
                if(priv->rom_table.mod_id == 0x0273)
                {
                  if( *(priv->rom_table.sen_type+len-1) == 'C' )
                  {
                    dev_err(&client->dev, "[ COLOR  ] [  %c ]\n", *(priv->rom_table.sen_type+len-1));
                    priv->model = IMX273_MODEL_COLOR;
                  }
                  else
                  {
                    dev_err(&client->dev, "[ MONO   ] [ B/W ]\n");
                    priv->model = IMX273_MODEL_MONOCHROME;
                  }
                }

            } // if(len > 0 && len < 17)
        }

        if(priv->model == IMX_MODEL_NONE)
        {
          dev_err(&client->dev, "%s(): Invalid sensor model=0x%04x\n", __func__, priv->rom_table.mod_id);
          return -ENODEV;
        }


//# sensor registers:
//004A:08 70          # chip ID high byte     = 0x7008    value = 0x02
//004C:07 70          # chip ID low byte      = 0x7007    value = 0x52
//004E:09 70          # chip revision         = 0x7009    value = 0x00
//
//#
//0050:00 70          # idle                  = 0x7000
//0052:14 60          # horizontal start high = 0x6014
//0054:13 60          # horizontal start low  = 0x6013
//0056:0F 60          # vertical start high   = 0x600f
//0058:0E 60          # vertical start low    = 0x600e
//005A:00 00          # horizontal end H      = 0x0000
//005C:00 00          # horizontal end L      = 0x0000
//005E:00 00          # vertical   end H      = 0x0000
//0060:00 00          # vertical   end L      = 0x0000
//0062:16 60          # hor. output width H   = 0x6016
//0064:15 60          # hor. output width L   = 0x6015
//0066:11 60          # ver. output height H  = 0x6011
//0068:10 60          # ver. output height L  = 0x6010
//006A:8F 02          # exposure H            = 0x028f
//006C:8E 02          # exposure M            = 0x028e
//006E:8D 02          # exposure L            = 0x028d
//0070:05 04          # gain high             = 0x0405
//0072:04 04          # gain low              = 0x0404
//#

#if DUMP_ROM_TABLE_REGS
{
    int i;
    dev_err(dev, "ROM table register dump:\n");
    for(i=0; i<56; i+=2)
    {
      dev_err(dev, "0x%02x: 0x%02x 0x%02x\n", i, (int)priv->rom_table.regs[i], (int)priv->rom_table.regs[i+1]);
    }
}
#endif


////................ Check model:
////        if(priv->rom_table.mod_id == 0x0273)
////        {
////          dev_err(dev, "%s(): IMX273 sensor OK\n", __func__);
////        }
////        else
////        {
////          dev_err(dev, "%s(): Invalid sensor 0x%x\n", __func__, priv->rom_table.mod_id);
////          return -EIO;
////        }
//
//#if MOD_ID
//        if(MOD_ID != priv->rom_table.mod_id)
//        {
//          dev_err(dev, "%s(): Invalid module id: E=0x%04x A=0x%04x\n", __func__, MOD_ID, priv->rom_table.mod_id);
//          err = -EIO;
//          goto done;
//        }
//        dev_err(dev, "%s(): Valid module id: 0x%04x\n", __func__, priv->rom_table.mod_id);
//#endif
//
//#if ERR_MOD_ID
//        dev_err(dev, "===> %s(): Invalid module id test !!\n", __func__);
//        err = -EIO;
//        goto done;
//#endif

// ???
      if(ext_trig_mode >= 1)
      {
        imx273_common_ops.numfrmfmts = ARRAY_SIZE(imx273_trig_frmfmt);
        imx273_common_ops.frmfmt_table = imx273_trig_frmfmt;

        tc_dev->s_data->numfmts = ARRAY_SIZE(imx273_trig_frmfmt);  // tc_dev->sensor_ops->numfrmfmts;
        tc_dev->s_data->frmfmt  = imx273_trig_frmfmt;             // tc_dev->sensor_ops->frmfmt_table;

        sensor_mode += 3;

        dev_err(dev, "%s(): sensor_mode=%d\n", __func__, sensor_mode);
      }

//      else
//      {
//        imx273_common_ops.numfrmfmts = ARRAY_SIZE(imx273_frmfmt);
//        imx273_common_ops.frmfmt_table = imx273_frmfmt;
//      }

//................. Reset VC MIPI sensor: Initialize FPGA: reset sensor registers to def. values
#if TRACE_IMX273_BOARD_SETUP
{
    dev_err(dev, "%s(): sensor_mode=%d\n", __func__, sensor_mode);
}
#endif
        priv->sensor_mode = sensor_mode;
        err = vc_mipi_reset(tc_dev, sensor_mode);
        if(err)
        {
          dev_err(dev, "%s(): vc_mipi_reset() error=%d\n", __func__, err);
          goto done;
        }


#if DUMP_CTL_REG_DATA
{
        int i;
        int reg_val[20];

        dev_err(&client->dev, "%s(): Module controller registers (0x%02x):\n", __func__, priv->fpga_addr);

//        addr = 0x100;
        i = 0;
        for(addr=0x100; addr<=0x110; addr++)
        {
          reg_val[i] = reg_read(priv->rom, addr);
//          dev_err(&client->dev, "0x%04x: %02x\n", addr, reg_val[i]);
          i++;
        }

        dev_err(&client->dev, "0x100-0x103: %02x %02x %02x %02x\n", reg_val[0],reg_val[1],reg_val[2],reg_val[3]);
        dev_err(&client->dev, "0x104-0x108: %02x %02x %02x %02x %02x\n", reg_val[4],reg_val[5],reg_val[6],reg_val[7],reg_val[8]);
        dev_err(&client->dev, "0x109-0x10C: %02x %02x %02x %02x\n", reg_val[9],reg_val[10],reg_val[11],reg_val[12]);
        dev_err(&client->dev, "0x10D-0x110: %02x %02x %02x %02x\n", reg_val[13],reg_val[14],reg_val[15],reg_val[16]);

}
//[    2.195918] imx273 7-001a: imx273_board_setup(): Module controller registers (0x10):
//[    2.208811] imx273 7-001a: 0x100-0x103: 00 80 01 08
//[    2.213749] imx273 7-001a: 0x104-0x108: 10 1a 00 00 00
//[    2.218978] imx273 7-001a: 0x109-0x10C: 10 27 00 00
//[    2.223926] imx273 7-001a: 0x10D-0x110: 3e ec 07 00

#endif

    }
    else
    {
        dev_err(&client->dev, "%s(): Error !!! VC FPGA not found !!!, fpga_addr=0x%02x\n", __func__, priv->fpga_addr);
        return -EIO;
    }

    ret = imx273_video_probe(client);
    if (ret < 0)
    {
      dev_err(dev, "%s(): imx273_video_probe() error=%d\n", __func__, ret);
      err = -EIO;
      goto done;
    }

#if TRACE_IMX273_BOARD_SETUP
{
    char *sen_models[4] =
    {
      "IMX_MODEL_NONE",
      "IMX273_MODEL_MONOCHROME",
      "IMX273_MODEL_COLOR",
//      "IMX297_MODEL_MONOCHROME",
//      "IMX297_MODEL_COLOR",
    };
    dev_err(&client->dev, "%s(): Sensor model=%s, err=%d\n", __func__, sen_models[priv->model], err);
}
#endif

#endif  // ]]] - end of VC_CODE

#if DUMP_IMX273_REGS
    imx273_dump_regs(priv);
#endif


//err_reg_probe:
    imx273_power_off(s_data);

err_power_on:
    if (pdata->mclk_name)
        camera_common_mclk_disable(s_data);

done:
    return err;
}

/****** imx273_open = Open device = 09.2020 *****************************/
static int imx273_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    dev_dbg(&client->dev, "%s:\n", __func__);
    return 0;
}

static const struct v4l2_subdev_internal_ops imx273_subdev_internal_ops = {
    .open = imx273_open,
};

static const struct of_device_id imx273_of_match[] = {
    { .compatible = "nvidia,imx273", },
    { },
};
MODULE_DEVICE_TABLE(of, imx273_of_match);

/****** imx273_probe = Probe = 09.2020 **********************************/
static int imx273_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
//    struct device_node *node = client->dev.of_node;
    struct tegracam_device *tc_dev;
    struct imx273 *priv;
    int err;
    const struct of_device_id *match;

//    dev_info(dev, "%s(): Probing v4l2 sensor at addr 0x%0x\n", __func__, client->addr); // , __DATE__, __TIME__);
    dev_err(dev, "%s(): Probing v4l2 sensor at addr 0x%0x - %s/%s\n", __func__, client->addr, __DATE__, __TIME__);
    fpga_addr = 0x10;    // def. FPGA i2c address

    match = of_match_device(imx273_of_match, dev);
    if (!match) {
        dev_err(dev, "No device match found\n");
        return -ENODEV;
    }
    dev_err(dev, "%s(): of_match_device() OK\n", __func__);

    if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
    {
        dev_err(dev, "%s(): !IS_ENABLED(CONFIG_OF) || !client->dev.of_node\n", __func__);
        return -EINVAL;
    }

    priv = devm_kzalloc(dev,
                sizeof(struct imx273), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    tc_dev = devm_kzalloc(dev,
                sizeof(struct tegracam_device), GFP_KERNEL);
    if (!tc_dev)
        return -ENOMEM;

    dev_info(dev, "%s(): devm_kzalloc() OK\n", __func__);

    priv->i2c_client = tc_dev->client = client;
    tc_dev->dev = dev;
    strncpy(tc_dev->name, "imx273", sizeof(tc_dev->name));
    tc_dev->dev_regmap_config = &imx273_regmap_config;
    tc_dev->sensor_ops = &imx273_common_ops;
    tc_dev->v4l2sd_internal_ops = &imx273_subdev_internal_ops;
    tc_dev->tcctrl_ops = &imx273_ctrl_ops;

    err = tegracam_device_register(tc_dev);
    if (err) {
        dev_err(dev, "tegra camera driver registration failed\n");
        return err;
    }
    dev_info(dev, "%s(): tegracam_device_register() OK\n", __func__);

    priv->tc_dev = tc_dev;
    priv->s_data = tc_dev->s_data;
    priv->subdev = &tc_dev->s_data->subdev;
    tegracam_set_privdata(tc_dev, (void *)priv);

#if IMX273_ENB_MUTEX
    mutex_init(&priv->mutex);
#endif

    priv->fpga_addr = fpga_addr;    // FPGA i2c address

    err = imx273_board_setup(priv);
    if (err) {
        dev_err(dev, "%s: imx273_board_setup() error=%d\n", __func__, err);
        return err;
    }
    dev_info(dev, "%s(): imx273_board_setup() OK\n", __func__);

    err = tegracam_v4l2subdev_register(tc_dev, true);
    if (err) {
        dev_err(dev, "tegra camera subdev registration failed\n");
        return err;
    }
    dev_info(dev, "%s(): tegracam_v4l2subdev_register() OK\n", __func__);

    priv->digital_gain  = IMX273_DIGITAL_GAIN_DEFAULT;
    priv->exposure_time = IMX273_DIGITAL_EXPOSURE_DEFAULT;
    priv->frame_rate    = IMX273_FRAME_RATE_DEFAULT;
//    priv->frame_length = IMX273_DEFAULT_FRAME_LENGTH;
    if(ext_trig_mode >= 0)
      priv->sensor_ext_trig = ext_trig_mode;    // ext. trigger flag: 0=no, 1=yes
    else
      priv->sensor_ext_trig = 0;    // ext. trigger flag: 0=no, 1=yes

//    priv->sen_clk = 54000000;     // clock-frequency: default=54Mhz imx183=72Mhz
    priv->sen_clk = IMX273_CLOCK_FREQUENCY;     // clock-frequency: default=54Mhz imx183=72Mhz
    priv->flash_output = flash_output;

//???
//    imx273_set_exposure(tc_dev, priv->exposure_time);
//    imx273_set_gain(tc_dev, priv->digital_gain);

{
    struct camera_common_data   *s_data = tc_dev->s_data;
    struct sensor_mode_properties *mode = &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

    mode->control_properties.default_gain         = IMX273_DIGITAL_GAIN_DEFAULT;
    mode->control_properties.default_exp_time.val = IMX273_DIGITAL_EXPOSURE_DEFAULT;
}


//    set_sensor_model(imx273_sensor_model);
    set_sensor_model("imx273");

//    dev_err(dev, "%s(): Detected imx273 sensor - %s/%s\n", __func__, __DATE__, __TIME__);

    switch(priv->model)
    {
      case IMX273_MODEL_MONOCHROME:
        priv->cam_mode = IMX273_MODE_1440X1080;
        dev_err(dev, "%s(): Detected imx273 sensor\n", __func__); // , __DATE__, __TIME__);
        break;

      case IMX273_MODEL_COLOR:
        priv->cam_mode = IMX273_MODE_1440X1080;
        dev_err(dev, "%s(): Detected imx273c sensor\n", __func__); // , __DATE__, __TIME__);
        break;

      default:
        dev_err(dev, "%s(): Unknown IMX sensor\n", __func__); // , __DATE__, __TIME__);
        break;
    }

    return 0;
}

/****** imx273_remove = Remove = 09.2020 ********************************/
static int imx273_remove(struct i2c_client *client)
{
    struct camera_common_data *s_data = to_camera_common_data(&client->dev);
    struct imx273 *priv = (struct imx273 *)s_data->priv;

    tegracam_v4l2subdev_unregister(priv->tc_dev);
//    ov5693_power_put(priv->tc_dev);
    tegracam_device_unregister(priv->tc_dev);

#if IMX273_ENB_MUTEX
    mutex_destroy(&priv->mutex);
#endif

    return 0;
}

static const struct i2c_device_id imx273_id[] = {
    { "imx273", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, imx273_id);



/****** i2c_driver = I2C driver = 09.2020 *******************************/
static struct i2c_driver imx273_i2c_driver = {
    .driver = {
        .name = "imx273",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(imx273_of_match),
    },
    .probe = imx273_probe,
    .remove = imx273_remove,
    .id_table = imx273_id,
};
module_i2c_driver(imx273_i2c_driver);

MODULE_VERSION("1.04");
MODULE_DESCRIPTION("Media Controller driver for IMX273");
//MODULE_AUTHOR("NVIDIA Corporation");
MODULE_AUTHOR("Vision Components GmbH <mipi-tech@vision-components.com>");
MODULE_LICENSE("GPL v2");

#if 0   // [[[
#endif   // ]]]
