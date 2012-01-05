/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include "rmi.h"

#define F11_MAX_NUM_OF_SENSORS		8
#define F11_MAX_NUM_OF_FINGERS		10
#define F11_MAX_NUM_OF_TOUCH_SHAPES	16

#define F11_REL_POS_MIN		-128
#define F11_REL_POS_MAX		127

#define F11_FINGER_STATE_MASK	0x03
#define F11_FINGER_STATE_SIZE	0x02
#define F11_FINGER_STATE_MASK_N(i) \
		(F11_FINGER_STATE_MASK << (i%4 * F11_FINGER_STATE_SIZE))

#define F11_FINGER_STATE_VAL_N(f_state, i) \
		(f_state >> (i%4 * F11_FINGER_STATE_SIZE))

#define F11_TOUCH_SHAPE_MASK	0x01
#define F11_TOUCH_SHAPE_SIZE	0x01

#define F11_CTRL_SENSOR_MAX_X_POS_OFFSET	6
#define F11_CTRL_SENSOR_MAX_Y_POS_OFFSET	8

#define F11_CEIL(x, y) (((x) + ((y)-1)) / (y))

struct f11_2d_device_query {
	union {
		struct {
			u8 nbr_of_sensors:3;
			u8 has_query9:1;
			u8 has_query11:1;
		};
		u8 f11_2d_query0;
	};

	u8 f11_2d_query9;

	union {
		struct {
			u8 has_z_tuning:1;
			u8 has_pos_interpolation_tuning:1;
			u8 has_w_tuning:1;
			u8 has_pitch_info:1;
			u8 has_default_finger_width:1;
			u8 has_segmentation_aggressiveness:1;
			u8 has_tx_rw_clip:1;
			u8 has_drumming_correction:1;
		};
		u8 f11_2d_query11;
	};
};

struct f11_2d_sensor_query {
	union {
		struct {
			/* query1 */
			u8 number_of_fingers:3;
			u8 has_rel:1;
			u8 has_abs:1;
			u8 has_gestures:1;
			u8 has_sensitivity_adjust:1;
			u8 configurable:1;
			/* query2 */
			u8 num_of_x_electrodes:7;
			/* query3 */
			u8 num_of_y_electrodes:7;
			/* query4 */
			u8 max_electrodes:7;
		};
		u8 f11_2d_query1__4[4];
	};

	union {
		struct {
			u8 abs_data_size:3;
			u8 has_anchored_finger:1;
			u8 has_adj_hyst:1;
			u8 has_dribble:1;
		};
		u8 f11_2d_query5;
	};

	u8 f11_2d_query6;

	union {
		struct {
			u8 has_single_tap:1;
			u8 has_tap_n_hold:1;
			u8 has_double_tap:1;
			u8 has_early_tap:1;
			u8 has_flick:1;
			u8 has_press:1;
			u8 has_pinch:1;
			u8 padding:1;

			u8 has_palm_det:1;
			u8 has_rotate:1;
			u8 has_touch_shapes:1;
			u8 has_scroll_zones:1;
			u8 has_individual_scroll_zones:1;
			u8 has_multi_finger_scroll:1;
		};
		u8 f11_2d_query7__8[2];
	};

	/* Empty */
	u8 f11_2d_query9;

	union {
		struct {
			u8 nbr_touch_shapes:5;
		};
		u8 f11_2d_query10;
	};
};

struct f11_2d_data_0 {
	u8 finger_n;
};

struct f11_2d_data_1_5 {
	u8 x_msb;
	u8 y_msb;
	u8 x_lsb:4;
	u8 y_lsb:4;
	u8 w_y:4;
	u8 w_x:4;
	u8 z;
};

struct f11_2d_data_6_7 {
	s8 delta_x;
	s8 delta_y;
};

struct f11_2d_data_8 {
	u8 single_tap:1;
	u8 tap_and_hold:1;
	u8 double_tap:1;
	u8 early_tap:1;
	u8 flick:1;
	u8 press:1;
	u8 pinch:1;
};

struct f11_2d_data_9 {
	u8 palm_detect:1;
	u8 rotate:1;
	u8 shape:1;
	u8 scrollzone:1;
	u8 finger_count:3;
};

struct f11_2d_data_10 {
	u8 pinch_motion;
};

struct f11_2d_data_10_12 {
	u8 x_flick_dist;
	u8 y_flick_dist;
	u8 flick_time;
};

struct f11_2d_data_11_12 {
	u8 motion;
	u8 finger_separation;
};

struct f11_2d_data_13 {
	u8 shape_n;
};

struct f11_2d_data_14_15 {
	u8 horizontal;
	u8 vertical;
};

struct f11_2d_data_14_17 {
	u8 x_low;
	u8 y_right;
	u8 x_upper;
	u8 y_left;
};

struct f11_2d_data {
	const struct f11_2d_data_0	*f_state;
	const struct f11_2d_data_1_5	*abs_pos;
	const struct f11_2d_data_6_7	*rel_pos;
	const struct f11_2d_data_8	*gest_1;
	const struct f11_2d_data_9	*gest_2;
	const struct f11_2d_data_10	*pinch;
	const struct f11_2d_data_10_12	*flick;
	const struct f11_2d_data_11_12	*rotate;
	const struct f11_2d_data_13	*shapes;
	const struct f11_2d_data_14_15	*multi_scroll;
	const struct f11_2d_data_14_17	*scroll_zones;
};

struct f11_2d_sensor {
	struct rmi_f11_2d_axis_alignment axis_align;
	struct f11_2d_sensor_query sens_query;
	struct f11_2d_data data;
	u16 max_x;
	u16 max_y;
	u8 nbr_fingers;
	u8 finger_tracker[F11_MAX_NUM_OF_FINGERS];
	u8 *data_pkt;
	int pkt_size;

	struct input_dev *input;
};

struct f11_data {
	struct f11_2d_device_query dev_query;
	struct f11_2d_sensor sensors[F11_MAX_NUM_OF_SENSORS];
};

enum finger_state_values {
	F11_NO_FINGER	= 0x00,
	F11_PRESENT	= 0x01,
	F11_INACCURATE	= 0x02,
	F11_RESERVED	= 0x03
};

static void rmi_f11_rel_pos_report(struct f11_2d_sensor *sensor, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	s8 x, y;
	s8 temp;

	x = data->rel_pos[n_finger].delta_x;
	y = data->rel_pos[n_finger].delta_y;

	x = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)x));
	y = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)y));

	if (axis_align->swap_axes) {
		temp = x;
		x = y;
		y = temp;
	}

	if (x || y) {
		input_report_rel(sensor->input, REL_X, x);
		input_report_rel(sensor->input, REL_Y, y);
	}
}

static void rmi_f11_abs_pos_report(struct f11_2d_sensor *sensor,
					u8 finger_state, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	int prev_state = sensor->finger_tracker[n_finger];
	int x, y, z;
	int w_x, w_y, w_max, w_min, orient;
	int temp;

	if (prev_state && !finger_state) {
		/* this is a release */
		x = y = z = w_max = w_min = orient = 0;
	} else if (!prev_state && !finger_state) {
		/* nothing to report */
		return;
	} else {
		x = ((data->abs_pos[n_finger].x_msb << 4) |
			data->abs_pos[n_finger].x_lsb);
		y = ((data->abs_pos[n_finger].y_msb << 4) |
			data->abs_pos[n_finger].y_lsb);
		z = data->abs_pos[n_finger].z;
		w_x = data->abs_pos[n_finger].w_x;
		w_y = data->abs_pos[n_finger].w_y;
		w_max = max(w_x, w_y);
		w_min = min(w_x, w_y);

		if (axis_align->swap_axes) {
			temp = x;
			x = y;
			y = temp;
			temp = w_x;
			w_x = w_y;
			w_y = temp;
		}

		orient = w_x > w_y ? 1 : 0;

		if (axis_align->flip_x)
			x = max(sensor->max_x - x, 0);

		if (axis_align->flip_y)
			y = max(sensor->max_y - y, 0);
	}

	pr_debug("%s: f_state[%d]:%d - x:%d y:%d z:%d w_max:%d w_min:%d\n",
		__func__, n_finger, finger_state, x, y, z, w_max, w_min);


#ifdef ABS_MT_PRESSURE
	input_report_abs(sensor->input, ABS_MT_PRESSURE, z);
#endif
	input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, w_max);
	input_report_abs(sensor->input, ABS_MT_TOUCH_MINOR, w_min);
	input_report_abs(sensor->input, ABS_MT_ORIENTATION, orient);
	input_report_abs(sensor->input, ABS_MT_POSITION_X, x);
	input_report_abs(sensor->input, ABS_MT_POSITION_Y, y);
	input_report_abs(sensor->input, ABS_MT_TRACKING_ID, n_finger);

	/* MT sync between fingers */
	input_mt_sync(sensor->input);
	sensor->finger_tracker[n_finger] = finger_state;
}

static void rmi_f11_finger_handler(struct f11_2d_sensor *sensor)
{
	const struct f11_2d_data_0 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 finger_pressed_count;
	u8 i;

	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		/* Possible of having 4 fingers per f_statet register */
		finger_state = (f_state[i >> 2].finger_n &
					F11_FINGER_STATE_MASK_N(i));
		finger_state = F11_FINGER_STATE_VAL_N(finger_state, i);

		if (finger_state == F11_RESERVED) {
			pr_err("%s: Invalid finger state[%d]:0x%02x.", __func__,
					i, finger_state);
			continue;
		} else if ((finger_state == F11_PRESENT) ||
				(finger_state == F11_INACCURATE)) {
			finger_pressed_count++;
		}

		if (sensor->data.abs_pos)
			rmi_f11_abs_pos_report(sensor, finger_state, i);

		if (sensor->data.rel_pos)
			rmi_f11_rel_pos_report(sensor, i);
	}
	input_report_key(sensor->input, BTN_TOUCH, finger_pressed_count);
	input_sync(sensor->input);
}

static inline int rmi_f11_2d_construct_data(struct f11_2d_sensor *sensor)
{
	struct f11_2d_sensor_query *query = &sensor->sens_query;
	struct f11_2d_data *data = &sensor->data;
	int i;

	sensor->nbr_fingers = (query->number_of_fingers == 5 ? 10 :
				query->number_of_fingers + 1);

	sensor->pkt_size = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs)
		sensor->pkt_size += (sensor->nbr_fingers * 5);

	if (query->has_rel)
		sensor->pkt_size +=  (sensor->nbr_fingers * 2);

	/* Check if F11_2D_Query7 is non-zero */
	if (query->f11_2d_query7__8[0])
		sensor->pkt_size += sizeof(u8);

	/* Check if F11_2D_Query7 or F11_2D_Query8 is non-zero */
	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1])
		sensor->pkt_size += sizeof(u8);

	if (query->has_pinch || query->has_flick || query->has_rotate) {
		sensor->pkt_size += 3;
		if (!query->has_flick)
			sensor->pkt_size--;
		if (!query->has_rotate)
			sensor->pkt_size--;
	}

	if (query->has_touch_shapes)
		sensor->pkt_size += F11_CEIL(query->nbr_touch_shapes + 1, 8);

	sensor->data_pkt = kzalloc(sensor->pkt_size, GFP_KERNEL);
	if (!sensor->data_pkt)
		return -ENOMEM;

	data->f_state = (struct f11_2d_data_0 *)sensor->data_pkt;
	i = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs) {
		data->abs_pos = (struct f11_2d_data_1_5 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 5);
	}

	if (query->has_rel) {
		data->rel_pos = (struct f11_2d_data_6_7 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 2);
	}

	if (query->f11_2d_query7__8[0]) {
		data->gest_1 = (struct f11_2d_data_8 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1]) {
		data->gest_2 = (struct f11_2d_data_9 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_pinch) {
		data->pinch = (struct f11_2d_data_10 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_flick) {
		if (query->has_pinch) {
			data->flick = (struct f11_2d_data_10_12 *)data->pinch;
			i += 2;
		} else {
			data->flick = (struct f11_2d_data_10_12 *)
					&sensor->data_pkt[i];
			i += 3;
		}
	}

	if (query->has_rotate) {
		if (query->has_flick) {
			data->rotate = (struct f11_2d_data_11_12 *)
					(data->flick + 1);
		} else {
			data->rotate = (struct f11_2d_data_11_12 *)
					&sensor->data_pkt[i];
			i += 2;
		}
	}

	if (query->has_touch_shapes)
		data->shapes = (struct f11_2d_data_13 *)&sensor->data_pkt[i];

	return 0;
}

static inline int rmi_f11_set_control_parameters(struct rmi_device *rmi_dev,
					struct f11_2d_sensor_query *query,
					struct rmi_f11_2d_ctrl *ctrl,
					u8 ctrl_base_addr)
{
	u8 offset = 0;
	int error;

	if (ctrl->ctrl0) {
		error = rmi_write(rmi_dev, ctrl_base_addr, ctrl->ctrl0->reg);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl1) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 1,
				  ctrl->ctrl1->reg);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl2) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 2, *ctrl->ctrl2);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl3) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 3, *ctrl->ctrl3);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl4) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 4, *ctrl->ctrl4);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl5) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 5, *ctrl->ctrl5);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl6__7) {
		error = rmi_write_block(rmi_dev, ctrl_base_addr + 6,
						&ctrl->ctrl6__7->regs[0],
						sizeof(ctrl->ctrl6__7->regs));
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl8__9) {
		error = rmi_write_block(rmi_dev, ctrl_base_addr + 8,
						&ctrl->ctrl8__9->regs[0],
						sizeof(ctrl->ctrl8__9->regs));
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl10) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 10,
						ctrl->ctrl10->reg);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl11) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 11,
						ctrl->ctrl11->reg);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl12 && ctrl->ctrl12_size && query->configurable) {
		if (ctrl->ctrl12_size > query->max_electrodes) {
			pr_err("%s: invalid cfg size:%d, should be < %d\n",
					__func__, ctrl->ctrl12_size,
							query->max_electrodes);
			return -EINVAL;
		}
		error = rmi_write_block(rmi_dev, ctrl_base_addr + 12,
						&ctrl->ctrl12->reg,
						ctrl->ctrl12_size);
		if (error < 0)
			return error;
		offset = query->max_electrodes - 1;
	}

	if (ctrl->ctrl14) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 14 + offset,
						ctrl->ctrl0->reg);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl15) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 15 + offset,
						*ctrl->ctrl15);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl16) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 16 + offset,
						*ctrl->ctrl16);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl17) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 17 + offset,
						*ctrl->ctrl17);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl18) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 18 + offset,
						*ctrl->ctrl18);
		if (error < 0)
			return error;
	}

	if (ctrl->ctrl19) {
		error = rmi_write(rmi_dev, ctrl_base_addr + 19 + offset,
						*ctrl->ctrl19);
		if (error < 0)
			return error;
	}

	return 0;
}

static inline int rmi_f11_get_query_parameters(struct rmi_device *rmi_dev,
			struct f11_2d_sensor_query *query, u8 query_base_addr)
{
	int query_size;
	int rc;

	rc = rmi_read_block(rmi_dev, query_base_addr, query->f11_2d_query1__4,
					sizeof(query->f11_2d_query1__4));
	if (rc < 0)
		return rc;

	query_size = rc;
	if (query->has_abs)
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query5);
	if (rc < 0)
		return rc;

	query_size++;
	if (query->has_rel)
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query6);
	if (rc < 0)
		return rc;

	query_size++;
	if (query->has_gestures)
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					query->f11_2d_query7__8,
					sizeof(query->f11_2d_query7__8));
	if (rc < 0)
		return rc;

	query_size += rc;
	if (query->has_touch_shapes) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query10);
		query_size++;
	}
	if (rc < 0)
		return rc;

	return query_size;
}

static int rmi_f11_init(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_device_platform_data *pdata;
	struct f11_data *f11;
	struct input_dev *input_dev;
	u8 query_offset;
	u8 query_base_addr;
	u8 control_base_addr;
	u16 max_x_pos, max_y_pos, temp;
	int rc;
	int i;

	f11 = kzalloc(sizeof(struct f11_data), GFP_KERNEL);
	if (!f11)
		return -ENOMEM;

	query_base_addr = fc->fd.query_base_addr;
	control_base_addr = fc->fd.control_base_addr;

	rc = rmi_read(rmi_dev, query_base_addr, &f11->dev_query.f11_2d_query0);
	if (rc < 0)
		goto err_free_data;

	query_offset = (query_base_addr + 1);
	/* Increase with one since number of sensors is zero based */
	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		rc = rmi_f11_get_query_parameters(rmi_dev,
					&f11->sensors[i].sens_query,
					query_offset);
		if (rc < 0)
			goto err_free_data;

		query_offset += rc;

		pdata = to_rmi_platform_data(rmi_dev);
		if (pdata)
			f11->sensors[i].axis_align = pdata->axis_align;

		if (pdata && pdata->f11_ctrl) {
			rc = rmi_f11_set_control_parameters(rmi_dev,
						&f11->sensors[i].sens_query,
						pdata->f11_ctrl,
						control_base_addr);
			if (rc < 0)
				goto err_free_data;
		}

		if (pdata && pdata->f11_ctrl &&
				pdata->f11_ctrl->ctrl6__7 &&
				pdata->f11_ctrl->ctrl8__9) {
			max_x_pos = pdata->f11_ctrl->ctrl6__7->sensor_max_x_pos;
			max_y_pos = pdata->f11_ctrl->ctrl8__9->sensor_max_y_pos;

		} else {
			rc = rmi_read_block(rmi_dev,
			  control_base_addr + F11_CTRL_SENSOR_MAX_X_POS_OFFSET,
			  (u8 *)&max_x_pos, sizeof(max_x_pos));
			if (rc < 0)
				goto err_free_data;

			rc = rmi_read_block(rmi_dev,
			  control_base_addr + F11_CTRL_SENSOR_MAX_Y_POS_OFFSET,
			  (u8 *)&max_y_pos, sizeof(max_y_pos));
			if (rc < 0)
				goto err_free_data;
		}

		if (pdata->axis_align.swap_axes) {
			temp = max_x_pos;
			max_x_pos = max_y_pos;
			max_y_pos = temp;
		}
		f11->sensors[i].max_x = max_x_pos;
		f11->sensors[i].max_y = max_y_pos;

		rc = rmi_f11_2d_construct_data(&f11->sensors[i]);
		if (rc < 0)
			goto err_free_data;

		input_dev = input_allocate_device();
		if (!input_dev) {
			rc = -ENOMEM;
			goto err_free_data;
		}

		f11->sensors[i].input = input_dev;
		input_dev->name = "rmi_f11";
		input_dev->phys = "rmi_f11/input0";
		input_dev->dev.parent = &rmi_dev->dev;
		input_set_drvdata(input_dev, f11);

		set_bit(EV_SYN, input_dev->evbit);
		set_bit(EV_KEY, input_dev->evbit);
		set_bit(EV_ABS, input_dev->evbit);
		set_bit(EV_REL, input_dev->evbit);

#ifdef ABS_MT_PRESSURE
		input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
						0, 15, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR,
						0, 15, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
						0, 1, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
						1, 10, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_POSITION_X,
						0, max_x_pos, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
						0, max_y_pos, 0, 0);

		rc = input_register_device(input_dev);
		if (rc < 0)
			goto err_free_input;
	}

	fc->data = f11;
	return 0;

err_free_input:
	for (; i > 0; i--)
		input_free_device(f11->sensors[i].input);
err_free_data:
	kfree(f11);
	return rc;
}

int rmi_f11_attention(struct rmi_function_container *fc, u8 irq_bits)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *f11 = fc->data;
	u8 data_base_addr = fc->fd.data_base_addr;
	int data_base_addr_offset = 0;
	int error;
	int i;

	for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++) {
		error = rmi_read_block(rmi_dev,
				data_base_addr + data_base_addr_offset,
				f11->sensors[i].data_pkt,
				f11->sensors[i].pkt_size);
		if (error < 0)
			return error;

		rmi_f11_finger_handler(&f11->sensors[i]);
		data_base_addr_offset += f11->sensors[i].pkt_size;
	}
	return 0;
}

static void rmi_f11_remove(struct rmi_function_container *fc)
{
	struct f11_data *data = fc->data;
	int i;

	for (i = 0; i < (data->dev_query.nbr_of_sensors + 1); i++)
		input_unregister_device(data->sensors[i].input);
	kfree(fc->data);
}

static struct rmi_function_handler function_handler = {
	.func = 0x11,
	.init = rmi_f11_init,
	.attention = rmi_f11_attention,
	.remove = rmi_f11_remove
};

static int __init rmi_f11_module_init(void)
{
	int error;

	error = rmi_register_function_driver(&function_handler);
	if (error < 0) {
		pr_err("%s: register failed!\n", __func__);
		return error;
	}

	return 0;
}

static void __exit rmi_f11_module_exit(void)
{
	rmi_unregister_function_driver(&function_handler);
}

module_init(rmi_f11_module_init);
module_exit(rmi_f11_module_exit);

MODULE_AUTHOR("Stefan Nilsson <stefan.nilsson@unixphere.com>");
MODULE_DESCRIPTION("RMI f11 module");
MODULE_LICENSE("GPL");

