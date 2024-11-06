package com.example.uavground.mavlink.dto;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.math.BigDecimal;

/**
 * 无人机信息对象 VehicleInfoDto
 */

@Data
@NoArgsConstructor
@AllArgsConstructor
public class VehicleInfoDto
{
    /** 无人机mac地址 */
    private String mac;

    /** 无人机速度 km/h */
    private Double vehicleSpeed;

    /** 无人机电量 */
    private Double vehicleSoc;

    /** 无人机经度 */
    private Double vehicleLong;

    /** 无人机纬度 */
    private Double vehicleLat;

    /** 无人机海拔高度 */
    private Double vehicleAlt;

}
