#include "vc_mipi_core.h"
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>

#define VERSION "0.1.0"

struct vc_device {
        struct v4l2_subdev sd;
        struct v4l2_ctrl_handler ctrl_handler;
        struct media_pad pad;
        // struct gpio_desc *power_gpio;
        int power_on;
        struct mutex mutex;

        struct vc_cam cam;
};

static inline struct vc_device *to_vc_device(struct v4l2_subdev *sd)
{
        return container_of(sd, struct vc_device, sd);
}

static inline struct vc_cam *to_vc_cam(struct v4l2_subdev *sd)
{
        struct vc_device *device = to_vc_device(sd);
        return &device->cam;
}


// --- v4l2_subdev_core_ops ---------------------------------------------------

static void vc_set_power(struct vc_device *device, int on)
{
        struct device *dev = &device->cam.ctrl.client_sen->dev;

        if (device->power_on == on)
                return;

        vc_dbg(dev, "%s(): Set power: %s\n", __func__, on ? "on" : "off");

        // if (device->power_gpio)
        //     gpiod_set_value_cansleep(device->power_gpio, on);

        // if (on == 1) {
        //         vc_core_wait_until_device_is_ready(&device->cam, 1000);
        // }
        device->power_on = on;
}

static int vc_sd_s_power(struct v4l2_subdev *sd, int on)
{
        struct vc_device * device = to_vc_device(sd);

        mutex_lock(&device->mutex);

        vc_set_power(to_vc_device(sd), on);

        mutex_unlock(&device->mutex);

        return 0;
}

static int __maybe_unused vc_suspend(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct vc_device *device = to_vc_device(sd);
        struct vc_state *state = &device->cam.state;

        vc_dbg(dev, "%s()\n", __func__);

        mutex_lock(&device->mutex);

        if (state->streaming)
                vc_sen_stop_stream(&device->cam);

        vc_set_power(device, 0);

        mutex_unlock(&device->mutex);

        return 0;
}

static int __maybe_unused vc_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct vc_device *device = to_vc_device(sd);

        vc_dbg(dev, "%s()\n", __func__);

        mutex_lock(&device->mutex);

        vc_set_power(device, 1);

        mutex_unlock(&device->mutex);

        return 0;
}

static int vc_sd_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *control)
{
        struct vc_cam *cam = to_vc_cam(sd);
        struct device *dev = vc_core_get_sen_device(cam);
        __u32 left, top;

        switch (control->id) {
        case V4L2_CID_EXPOSURE:
                return vc_sen_set_exposure(cam, control->value);

        case V4L2_CID_GAIN:
                return vc_sen_set_gain(cam, control->value);

        case V4L2_CID_BLACK_LEVEL:
                return vc_sen_set_blacklevel(cam, control->value);

        case V4L2_CID_TRIGGER_MODE:
                return vc_mod_set_trigger_mode(cam, control->value);

        case V4L2_CID_FLASH_MODE:
                return vc_mod_set_io_mode(cam, control->value);

        case V4L2_CID_FRAME_RATE:
                return vc_core_set_framerate(cam, control->value);

        case V4L2_CID_SINGLE_TRIGGER:
                return vc_mod_set_single_trigger(cam);

        case V4L2_CID_BINNING_MODE:
                return vc_sen_set_binning_mode(cam, control->value);

        case V4L2_CID_ROI_POSITION:
                left = control->value/10000;
                top = control->value - left*10000;
                return vc_core_set_frame_position(cam, left, top);

        default:
                vc_warn(dev, "%s(): Unkown control 0x%08x\n", __func__, control->id);
                return -EINVAL;
        }

        return 0;
}

// --- v4l2_subdev_video_ops ---------------------------------------------------

static int vc_sd_s_stream(struct v4l2_subdev *sd, int enable)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct vc_state *state = &cam->state;
        struct device *dev = sd->dev;
        int reset = 0;
        int ret = 0;

        vc_dbg(dev, "%s(): Set streaming: %s\n", __func__, enable ? "on" : "off");

        if (state->streaming == enable)
                return 0;

        mutex_lock(&device->mutex);
        if (enable) {
                ret = pm_runtime_get_sync(dev);
                if (ret < 0) {
                        pm_runtime_put_noidle(dev);
                        mutex_unlock(&device->mutex);
                        return ret;
                }

                ret  = vc_mod_set_mode(cam, &reset);
                ret |= vc_sen_set_roi(cam);
                if (!ret && reset) {
                        ret |= vc_sen_set_exposure(cam, cam->state.exposure);
                        ret |= vc_sen_set_gain(cam, cam->state.gain);
                        ret |= vc_sen_set_blacklevel(cam, cam->state.blacklevel);
                }

                ret = vc_sen_start_stream(cam);
                if (ret) {
                        enable = 0;
                        vc_sen_stop_stream(cam);
                        pm_runtime_mark_last_busy(dev);
                        pm_runtime_put_autosuspend(dev);
                }

        } else {
                vc_sen_stop_stream(cam);
                pm_runtime_mark_last_busy(dev);
                pm_runtime_put_autosuspend(dev);
        }

        state->streaming = enable;
        mutex_unlock(&device->mutex);

        return ret;
}

// --- v4l2_subdev_pad_ops ---------------------------------------------------

static int vc_sd_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct v4l2_mbus_framefmt *mf = &format->format;
        struct vc_frame* frame = NULL;

        mutex_lock(&device->mutex);

        mf->code = vc_core_get_format(cam);
        frame = vc_core_get_frame(cam);
        mf->width = frame->width;
        mf->height = frame->height;

        mutex_unlock(&device->mutex);

        return 0;
}

static int vc_sd_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_format *format)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);
        struct v4l2_mbus_framefmt *mf = &format->format;

        mutex_lock(&device->mutex);

        vc_core_set_format(cam, mf->code);
        vc_core_set_frame_size(cam, mf->width, mf->height);

        mutex_unlock(&device->mutex);

        return 0;
}

int vc_sd_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, struct v4l2_subdev_mbus_code_enum *code)
{
        struct vc_device *device = to_vc_device(sd);
        struct vc_cam *cam = to_vc_cam(sd);

        mutex_lock(&device->mutex);

        code->code = vc_core_get_format(cam);

        mutex_unlock(&device->mutex);

        return 0;
}

// --- v4l2_ctrl_ops ---------------------------------------------------

int vc_ctrl_s_ctrl(struct v4l2_ctrl *ctrl)
{
        struct vc_device *device = container_of(ctrl->handler, struct vc_device, ctrl_handler);
        // struct i2c_client *client = device->cam.ctrl.client_sen;
        struct v4l2_control control;

        // V4L2 controls values will be applied only when power is already up
        // if (!pm_runtime_get_if_in_use(&client->dev))
        //     return 0;

        mutex_lock(&device->mutex);

        control.id = ctrl->id;
        control.value = ctrl->val;
        vc_sd_s_ctrl(&device->sd, &control);

        mutex_unlock(&device->mutex);

        return 0;
}



// *** Initialisation *********************************************************

// static void vc_setup_power_gpio(struct vc_device *device)
// {
//         struct device *dev = &device->cam.ctrl.client_sen->dev;

//         device->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_HIGH);
//         if (IS_ERR(device->power_gpio)) {
//                 vc_err(dev, "%s(): Failed to setup power-gpio\n", __func__);
//                 device->power_gpio = NULL;
//         }
// }

static int vc_check_hwcfg(struct vc_cam *cam, struct device *dev)
{
        struct fwnode_handle *endpoint;
        struct v4l2_fwnode_endpoint ep_cfg = {
                .bus_type = V4L2_MBUS_CSI2_DPHY
        };
        int ret = -EINVAL;

        endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
        if (!endpoint) {
                dev_err(dev, "Endpoint node not found!\n");
                return -EINVAL;
        }

        if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
                dev_err(dev, "Could not parse endpoint!\n");
                goto error_out;
        }

        /* Set and check the number of MIPI CSI2 data lanes */
        ret = vc_core_set_num_lanes(cam, ep_cfg.bus.mipi_csi2.num_data_lanes);;

error_out:
        v4l2_fwnode_endpoint_free(&ep_cfg);
        fwnode_handle_put(endpoint);

        return ret;
}

static const struct v4l2_subdev_core_ops vc_core_ops = {
        .s_power = vc_sd_s_power,
};

static const struct v4l2_subdev_video_ops vc_video_ops = {
        .s_stream = vc_sd_s_stream,
};

static const struct v4l2_subdev_pad_ops vc_pad_ops = {
        .get_fmt = vc_sd_get_fmt,
        .set_fmt = vc_sd_set_fmt,
        .enum_mbus_code = vc_sd_enum_mbus_code,
};

static const struct v4l2_subdev_ops vc_subdev_ops = {
        .core = &vc_core_ops,
        .video = &vc_video_ops,
        .pad = &vc_pad_ops,
};

static const struct v4l2_ctrl_ops vc_ctrl_ops = {
        .s_ctrl = vc_ctrl_s_ctrl,
};

static int vc_ctrl_init_ctrl(struct vc_device *device, struct v4l2_ctrl_handler *hdl, int id, struct vc_control* control)
{
        struct i2c_client *client = device->cam.ctrl.client_sen;
        struct device *dev = &client->dev;
        struct v4l2_ctrl *ctrl;

        ctrl = v4l2_ctrl_new_std(&device->ctrl_handler, &vc_ctrl_ops, id, control->min, control->max, 1, control->def);
        if (ctrl == NULL) {
                vc_err(dev, "%s(): Failed to init 0x%08x ctrl\n", __func__, id);
                return -EIO;
        }

        return 0;
}

static int vc_ctrl_init_custom_ctrl(struct vc_device *device, struct v4l2_ctrl_handler *hdl, const struct v4l2_ctrl_config *config)
{
        struct i2c_client *client = device->cam.ctrl.client_sen;
        struct device *dev = &client->dev;
        struct v4l2_ctrl *ctrl;

        ctrl = v4l2_ctrl_new_custom(&device->ctrl_handler, config, NULL);
        if (ctrl == NULL) {
                vc_err(dev, "%s(): Failed to init 0x%08x ctrl\n", __func__, config->id);
                return -EIO;
        }

        return 0;
}

static const struct v4l2_ctrl_config ctrl_trigger_mode = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_TRIGGER_MODE,
        .name = "Trigger Mode",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 7,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_flash_mode = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_FLASH_MODE,
        .name = "Flash Mode",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 1,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_frame_rate = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_FRAME_RATE,
        .name = "Frame Rate",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 1000000,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_single_trigger = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_SINGLE_TRIGGER,
        .name = "Single Trigger",
        .type = V4L2_CTRL_TYPE_BUTTON,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 1,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_binning_mode = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_BINNING_MODE,
        .name = "Binning Mode",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 4,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config ctrl_roi_position = {
        .ops = &vc_ctrl_ops,
        .id = V4L2_CID_ROI_POSITION,
        .name = "Roi Position",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
        .min = 0,
        .max = 99999999,
        .step = 1,
        .def = 0,
};

static int vc_sd_init(struct vc_device *device)
{
        struct i2c_client *client = device->cam.ctrl.client_sen;
        struct device *dev = &client->dev;
        int ret;

        // Initializes the subdevice
        v4l2_i2c_subdev_init(&device->sd, client, &vc_subdev_ops);

        // Initialize the handler
        ret = v4l2_ctrl_handler_init(&device->ctrl_handler, 3);
        if (ret) {
                vc_err(dev, "%s(): Failed to init control handler\n", __func__);
                return ret;
        }
        // Hook the control handler into the driver
        device->sd.ctrl_handler = &device->ctrl_handler;

        // Add controls
        ret |= vc_ctrl_init_ctrl(device, &device->ctrl_handler, V4L2_CID_EXPOSURE, &device->cam.ctrl.exposure);
        ret |= vc_ctrl_init_ctrl(device, &device->ctrl_handler, V4L2_CID_GAIN, &device->cam.ctrl.gain);
        ret |= vc_ctrl_init_ctrl(device, &device->ctrl_handler, V4L2_CID_BLACK_LEVEL, &device->cam.ctrl.blacklevel);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_trigger_mode);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_flash_mode);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_frame_rate);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_single_trigger);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_binning_mode);
        ret |= vc_ctrl_init_custom_ctrl(device, &device->ctrl_handler, &ctrl_roi_position);

        return 0;
}

static int vc_link_setup(struct media_entity *entity, const struct media_pad *local, const struct media_pad *remote,
                         __u32 flags)
{
        return 0;
}

static const struct media_entity_operations vc_sd_media_ops = {
        .link_setup = vc_link_setup,
};

static int vc_probe(struct i2c_client *client)
{
        struct device *dev = &client->dev;
        struct vc_device *device;
        struct vc_cam *cam;
        int ret;

        vc_notice(dev, "%s(): Probing UNIVERSAL VC MIPI Driver (v%s)\n", __func__, VERSION);

        device = devm_kzalloc(dev, sizeof(*device), GFP_KERNEL);
        if (!device)
                return -ENOMEM;

        cam = &device->cam;
        cam->ctrl.client_sen = client;

        // vc_setup_power_gpio(device);
        vc_set_power(device, 1);

        ret = vc_core_init(cam, client);
        if (ret)
                goto error_power_off;

        ret = vc_check_hwcfg(cam, dev);
        if (ret)
                goto error_power_off;

        mutex_init(&device->mutex);
        ret = vc_sd_init(device);
        if (ret)
                goto error_handler_free;

        device->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
        device->pad.flags = MEDIA_PAD_FL_SOURCE;
        device->sd.entity.ops = &vc_sd_media_ops;
        device->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
        ret = media_entity_pads_init(&device->sd.entity, 1, &device->pad);
        if (ret)
                goto error_handler_free;

        ret = v4l2_async_register_subdev_sensor(&device->sd);
        if (ret)
                goto error_media_entity;

        pm_runtime_get_noresume(dev);
        pm_runtime_set_active(dev);
        pm_runtime_enable(dev);
        pm_runtime_set_autosuspend_delay(dev, 2000);
        pm_runtime_use_autosuspend(dev);
        pm_runtime_mark_last_busy(dev);
        pm_runtime_put_autosuspend(dev);

        return 0;

error_media_entity:
        media_entity_cleanup(&device->sd.entity);

error_handler_free:
        v4l2_ctrl_handler_free(&device->ctrl_handler);
        mutex_destroy(&device->mutex);

error_power_off:
        pm_runtime_disable(dev);
        pm_runtime_set_suspended(dev);
        pm_runtime_put_noidle(dev);
        vc_set_power(device, 0);
        return ret;
}

static int vc_remove(struct i2c_client *client)
{
        struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct vc_device *device = to_vc_device(sd);

        v4l2_async_unregister_subdev(&device->sd);
        media_entity_cleanup(&device->sd.entity);
        v4l2_ctrl_handler_free(&device->ctrl_handler);
        pm_runtime_disable(&client->dev);
        mutex_destroy(&device->mutex);

        pm_runtime_get_sync(&client->dev);
        pm_runtime_disable(&client->dev);
        pm_runtime_set_suspended(&client->dev);
        pm_runtime_put_noidle(&client->dev);

        vc_set_power(device, 0);

        return 0;
}

static const struct dev_pm_ops vc_pm_ops = {
        SET_SYSTEM_SLEEP_PM_OPS(vc_suspend, vc_resume)
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id vc_acpi_ids[] = {
        {"VCMIPICAM"},
        {}
};

MODULE_DEVICE_TABLE(acpi, vc_acpi_ids);
#endif

static const struct i2c_device_id vc_id[] = {
        { "vc-mipi-cam", 0 },
        { /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, vc_id);

static const struct of_device_id vc_dt_ids[] = {
        { .compatible = "vc,vc_mipi" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vc_dt_ids);

static struct i2c_driver vc_i2c_driver = {
        .driver = {
                .name = "vc-mipi-cam",
                .pm = &vc_pm_ops,
                .acpi_match_table = ACPI_PTR(vc_acpi_ids),
                .of_match_table = vc_dt_ids,
        },
        .id_table = vc_id,
        .probe_new = vc_probe,
        .remove   = vc_remove,
};

module_i2c_driver(vc_i2c_driver);

MODULE_VERSION(VERSION);
MODULE_DESCRIPTION("Vision Components GmbH - VC MIPI CSI-2 driver");
MODULE_AUTHOR("Peter Martienssen, Liquify Consulting <peter.martienssen@liquify-consulting.de>");
MODULE_LICENSE("GPL v2");
