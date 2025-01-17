/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE CAMERA_TRACKING_GEO_STATUS PACKING
package com.MAVLink.common;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
import com.MAVLink.Messages.Units;
import com.MAVLink.Messages.Description;

/**
 * Camera tracking status, sent while in active tracking. Use MAV_CMD_SET_MESSAGE_INTERVAL to define message interval.
 */
public class msg_camera_tracking_geo_status extends MAVLinkMessage {

    public static final int MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS = 276;
    public static final int MAVLINK_MSG_LENGTH = 50;
    private static final long serialVersionUID = MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;

    
    /**
     * Latitude of tracked object
     */
    @Description("Latitude of tracked object")
    @Units("degE7")
    public int lat;
    
    /**
     * Longitude of tracked object
     */
    @Description("Longitude of tracked object")
    @Units("degE7")
    public int lon;
    
    /**
     * Altitude of tracked object(AMSL, WGS84)
     */
    @Description("Altitude of tracked object(AMSL, WGS84)")
    @Units("m")
    public float alt;
    
    /**
     * Horizontal accuracy. NAN if unknown
     */
    @Description("Horizontal accuracy. NAN if unknown")
    @Units("m")
    public float h_acc;
    
    /**
     * Vertical accuracy. NAN if unknown
     */
    @Description("Vertical accuracy. NAN if unknown")
    @Units("m")
    public float v_acc;
    
    /**
     * North velocity of tracked object. NAN if unknown
     */
    @Description("North velocity of tracked object. NAN if unknown")
    @Units("m/s")
    public float vel_n;
    
    /**
     * East velocity of tracked object. NAN if unknown
     */
    @Description("East velocity of tracked object. NAN if unknown")
    @Units("m/s")
    public float vel_e;
    
    /**
     * Down velocity of tracked object. NAN if unknown
     */
    @Description("Down velocity of tracked object. NAN if unknown")
    @Units("m/s")
    public float vel_d;
    
    /**
     * Velocity accuracy. NAN if unknown
     */
    @Description("Velocity accuracy. NAN if unknown")
    @Units("m/s")
    public float vel_acc;
    
    /**
     * Distance between camera and tracked object. NAN if unknown
     */
    @Description("Distance between camera and tracked object. NAN if unknown")
    @Units("m")
    public float dist;
    
    /**
     * Heading in radians, in NED. NAN if unknown
     */
    @Description("Heading in radians, in NED. NAN if unknown")
    @Units("rad")
    public float hdg;
    
    /**
     * Accuracy of heading, in NED. NAN if unknown
     */
    @Description("Accuracy of heading, in NED. NAN if unknown")
    @Units("rad")
    public float hdg_acc;
    
    /**
     * Current tracking status
     */
    @Description("Current tracking status")
    @Units("")
    public short tracking_status;
    
    /**
     * Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
     */
    @Description("Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).")
    @Units("")
    public short camera_device_id;
    

    /**
     * Generates the payload for a mavlink message for a message of this type
     * @return
     */
    @Override
    public MAVLinkPacket pack() {
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH,isMavlink2);
        packet.sysid = sysid;
        packet.compid = compid;
        packet.msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;

        packet.payload.putInt(lat);
        packet.payload.putInt(lon);
        packet.payload.putFloat(alt);
        packet.payload.putFloat(h_acc);
        packet.payload.putFloat(v_acc);
        packet.payload.putFloat(vel_n);
        packet.payload.putFloat(vel_e);
        packet.payload.putFloat(vel_d);
        packet.payload.putFloat(vel_acc);
        packet.payload.putFloat(dist);
        packet.payload.putFloat(hdg);
        packet.payload.putFloat(hdg_acc);
        packet.payload.putUnsignedByte(tracking_status);
        
        if (isMavlink2) {
             packet.payload.putUnsignedByte(camera_device_id);
            
        }
        return packet;
    }

    /**
     * Decode a camera_tracking_geo_status message into this class fields
     *
     * @param payload The message to decode
     */
    @Override
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();

        this.lat = payload.getInt();
        this.lon = payload.getInt();
        this.alt = payload.getFloat();
        this.h_acc = payload.getFloat();
        this.v_acc = payload.getFloat();
        this.vel_n = payload.getFloat();
        this.vel_e = payload.getFloat();
        this.vel_d = payload.getFloat();
        this.vel_acc = payload.getFloat();
        this.dist = payload.getFloat();
        this.hdg = payload.getFloat();
        this.hdg_acc = payload.getFloat();
        this.tracking_status = payload.getUnsignedByte();
        
        if (isMavlink2) {
             this.camera_device_id = payload.getUnsignedByte();
            
        }
    }

    /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_camera_tracking_geo_status() {
        this.msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;
    }

    /**
     * Constructor for a new message, initializes msgid and all payload variables
     */
    public msg_camera_tracking_geo_status( int lat, int lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc, short tracking_status, short camera_device_id) {
        this.msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;

        this.lat = lat;
        this.lon = lon;
        this.alt = alt;
        this.h_acc = h_acc;
        this.v_acc = v_acc;
        this.vel_n = vel_n;
        this.vel_e = vel_e;
        this.vel_d = vel_d;
        this.vel_acc = vel_acc;
        this.dist = dist;
        this.hdg = hdg;
        this.hdg_acc = hdg_acc;
        this.tracking_status = tracking_status;
        this.camera_device_id = camera_device_id;
        
    }

    /**
     * Constructor for a new message, initializes everything
     */
    public msg_camera_tracking_geo_status( int lat, int lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc, short tracking_status, short camera_device_id, int sysid, int compid, boolean isMavlink2) {
        this.msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;
        this.sysid = sysid;
        this.compid = compid;
        this.isMavlink2 = isMavlink2;

        this.lat = lat;
        this.lon = lon;
        this.alt = alt;
        this.h_acc = h_acc;
        this.v_acc = v_acc;
        this.vel_n = vel_n;
        this.vel_e = vel_e;
        this.vel_d = vel_d;
        this.vel_acc = vel_acc;
        this.dist = dist;
        this.hdg = hdg;
        this.hdg_acc = hdg_acc;
        this.tracking_status = tracking_status;
        this.camera_device_id = camera_device_id;
        
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a mavlink packet
     *
     */
    public msg_camera_tracking_geo_status(MAVLinkPacket mavLinkPacket) {
        this.msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;

        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.isMavlink2 = mavLinkPacket.isMavlink2;
        unpack(mavLinkPacket.payload);
    }

                                
    /**
     * Returns a string with the MSG name and data
     */
    @Override
    public String toString() {
        return "MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS - sysid:"+sysid+" compid:"+compid+" lat:"+lat+" lon:"+lon+" alt:"+alt+" h_acc:"+h_acc+" v_acc:"+v_acc+" vel_n:"+vel_n+" vel_e:"+vel_e+" vel_d:"+vel_d+" vel_acc:"+vel_acc+" dist:"+dist+" hdg:"+hdg+" hdg_acc:"+hdg_acc+" tracking_status:"+tracking_status+" camera_device_id:"+camera_device_id+"";
    }

    /**
     * Returns a human-readable string of the name of the message
     */
    @Override
    public String name() {
        return "MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS";
    }
}
        