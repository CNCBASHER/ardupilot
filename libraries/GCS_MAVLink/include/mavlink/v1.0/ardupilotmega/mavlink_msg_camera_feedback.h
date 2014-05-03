// MESSAGE CAMERA_FEEDBACK PACKING

#define MAVLINK_MSG_ID_CAMERA_FEEDBACK 179

typedef struct __mavlink_camera_feedback_t
{
 uint64_t time_usec; ///< Image timestamp
 float lat; ///< Latitude of capture point
 float lng; ///< Longitude of capture point
 float alt; ///< Altitude of capture point
 float roll; ///< Roll angle (degrees)
 float pitch; ///< Pitch angle (degrees)
 float heading; ///< Heading (degrees)
 float fov; ///< FOV (horz degrees)
 uint8_t cam_idx; ///< Camera ID
 uint8_t img_idx; ///< Image index
} mavlink_camera_feedback_t;

#define MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN 38
#define MAVLINK_MSG_ID_179_LEN 38

#define MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC 37
#define MAVLINK_MSG_ID_179_CRC 37



#define MAVLINK_MESSAGE_INFO_CAMERA_FEEDBACK { \
	"CAMERA_FEEDBACK", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_camera_feedback_t, time_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_feedback_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_feedback_t, lng) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_feedback_t, alt) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_feedback_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_feedback_t, pitch) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_camera_feedback_t, heading) }, \
         { "fov", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_camera_feedback_t, fov) }, \
         { "cam_idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_camera_feedback_t, cam_idx) }, \
         { "img_idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_camera_feedback_t, img_idx) }, \
         } \
}


/**
 * @brief Pack a camera_feedback message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cam_idx Camera ID
 * @param img_idx Image index
 * @param time_usec Image timestamp
 * @param lat Latitude of capture point
 * @param lng Longitude of capture point
 * @param alt Altitude of capture point
 * @param roll Roll angle (degrees)
 * @param pitch Pitch angle (degrees)
 * @param heading Heading (degrees)
 * @param fov FOV (horz degrees)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_feedback_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t cam_idx, uint8_t img_idx, uint64_t time_usec, float lat, float lng, float alt, float roll, float pitch, float heading, float fov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lng);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, heading);
	_mav_put_float(buf, 32, fov);
	_mav_put_uint8_t(buf, 36, cam_idx);
	_mav_put_uint8_t(buf, 37, img_idx);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#else
	mavlink_camera_feedback_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lng = lng;
	packet.alt = alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.heading = heading;
	packet.fov = fov;
	packet.cam_idx = cam_idx;
	packet.img_idx = img_idx;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAMERA_FEEDBACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif
}

/**
 * @brief Pack a camera_feedback message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cam_idx Camera ID
 * @param img_idx Image index
 * @param time_usec Image timestamp
 * @param lat Latitude of capture point
 * @param lng Longitude of capture point
 * @param alt Altitude of capture point
 * @param roll Roll angle (degrees)
 * @param pitch Pitch angle (degrees)
 * @param heading Heading (degrees)
 * @param fov FOV (horz degrees)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_feedback_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t cam_idx,uint8_t img_idx,uint64_t time_usec,float lat,float lng,float alt,float roll,float pitch,float heading,float fov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lng);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, heading);
	_mav_put_float(buf, 32, fov);
	_mav_put_uint8_t(buf, 36, cam_idx);
	_mav_put_uint8_t(buf, 37, img_idx);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#else
	mavlink_camera_feedback_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lng = lng;
	packet.alt = alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.heading = heading;
	packet.fov = fov;
	packet.cam_idx = cam_idx;
	packet.img_idx = img_idx;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAMERA_FEEDBACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif
}

/**
 * @brief Encode a camera_feedback struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_feedback_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_feedback_t* camera_feedback)
{
	return mavlink_msg_camera_feedback_pack(system_id, component_id, msg, camera_feedback->cam_idx, camera_feedback->img_idx, camera_feedback->time_usec, camera_feedback->lat, camera_feedback->lng, camera_feedback->alt, camera_feedback->roll, camera_feedback->pitch, camera_feedback->heading, camera_feedback->fov);
}

/**
 * @brief Encode a camera_feedback struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_feedback_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_feedback_t* camera_feedback)
{
	return mavlink_msg_camera_feedback_pack_chan(system_id, component_id, chan, msg, camera_feedback->cam_idx, camera_feedback->img_idx, camera_feedback->time_usec, camera_feedback->lat, camera_feedback->lng, camera_feedback->alt, camera_feedback->roll, camera_feedback->pitch, camera_feedback->heading, camera_feedback->fov);
}

/**
 * @brief Send a camera_feedback message
 * @param chan MAVLink channel to send the message
 *
 * @param cam_idx Camera ID
 * @param img_idx Image index
 * @param time_usec Image timestamp
 * @param lat Latitude of capture point
 * @param lng Longitude of capture point
 * @param alt Altitude of capture point
 * @param roll Roll angle (degrees)
 * @param pitch Pitch angle (degrees)
 * @param heading Heading (degrees)
 * @param fov FOV (horz degrees)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_feedback_send(mavlink_channel_t chan, uint8_t cam_idx, uint8_t img_idx, uint64_t time_usec, float lat, float lng, float alt, float roll, float pitch, float heading, float fov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lng);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, heading);
	_mav_put_float(buf, 32, fov);
	_mav_put_uint8_t(buf, 36, cam_idx);
	_mav_put_uint8_t(buf, 37, img_idx);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif
#else
	mavlink_camera_feedback_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lng = lng;
	packet.alt = alt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.heading = heading;
	packet.fov = fov;
	packet.cam_idx = cam_idx;
	packet.img_idx = img_idx;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_feedback_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t cam_idx, uint8_t img_idx, uint64_t time_usec, float lat, float lng, float alt, float roll, float pitch, float heading, float fov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lng);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, roll);
	_mav_put_float(buf, 24, pitch);
	_mav_put_float(buf, 28, heading);
	_mav_put_float(buf, 32, fov);
	_mav_put_uint8_t(buf, 36, cam_idx);
	_mav_put_uint8_t(buf, 37, img_idx);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif
#else
	mavlink_camera_feedback_t *packet = (mavlink_camera_feedback_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->lat = lat;
	packet->lng = lng;
	packet->alt = alt;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->heading = heading;
	packet->fov = fov;
	packet->cam_idx = cam_idx;
	packet->img_idx = img_idx;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, (const char *)packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, (const char *)packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CAMERA_FEEDBACK UNPACKING


/**
 * @brief Get field cam_idx from camera_feedback message
 *
 * @return Camera ID
 */
static inline uint8_t mavlink_msg_camera_feedback_get_cam_idx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field img_idx from camera_feedback message
 *
 * @return Image index
 */
static inline uint8_t mavlink_msg_camera_feedback_get_img_idx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field time_usec from camera_feedback message
 *
 * @return Image timestamp
 */
static inline uint64_t mavlink_msg_camera_feedback_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from camera_feedback message
 *
 * @return Latitude of capture point
 */
static inline float mavlink_msg_camera_feedback_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field lng from camera_feedback message
 *
 * @return Longitude of capture point
 */
static inline float mavlink_msg_camera_feedback_get_lng(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field alt from camera_feedback message
 *
 * @return Altitude of capture point
 */
static inline float mavlink_msg_camera_feedback_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field roll from camera_feedback message
 *
 * @return Roll angle (degrees)
 */
static inline float mavlink_msg_camera_feedback_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch from camera_feedback message
 *
 * @return Pitch angle (degrees)
 */
static inline float mavlink_msg_camera_feedback_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field heading from camera_feedback message
 *
 * @return Heading (degrees)
 */
static inline float mavlink_msg_camera_feedback_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field fov from camera_feedback message
 *
 * @return FOV (horz degrees)
 */
static inline float mavlink_msg_camera_feedback_get_fov(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a camera_feedback message into a struct
 *
 * @param msg The message to decode
 * @param camera_feedback C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_feedback_decode(const mavlink_message_t* msg, mavlink_camera_feedback_t* camera_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP
	camera_feedback->time_usec = mavlink_msg_camera_feedback_get_time_usec(msg);
	camera_feedback->lat = mavlink_msg_camera_feedback_get_lat(msg);
	camera_feedback->lng = mavlink_msg_camera_feedback_get_lng(msg);
	camera_feedback->alt = mavlink_msg_camera_feedback_get_alt(msg);
	camera_feedback->roll = mavlink_msg_camera_feedback_get_roll(msg);
	camera_feedback->pitch = mavlink_msg_camera_feedback_get_pitch(msg);
	camera_feedback->heading = mavlink_msg_camera_feedback_get_heading(msg);
	camera_feedback->fov = mavlink_msg_camera_feedback_get_fov(msg);
	camera_feedback->cam_idx = mavlink_msg_camera_feedback_get_cam_idx(msg);
	camera_feedback->img_idx = mavlink_msg_camera_feedback_get_img_idx(msg);
#else
	memcpy(camera_feedback, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif
}
