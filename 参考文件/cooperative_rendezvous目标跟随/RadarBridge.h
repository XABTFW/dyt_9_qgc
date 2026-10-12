/****************************************************************************
 *
 * RadarBridge
 *
 * 在 QGC 内接收"反无雷达"协议帧(UDP), 解码后编码成 MAVLink UAV_INFO(id=12921),
 * 通过 QGC 到飞控(PX4)的现有链路发送给飞控。
 *
 * 数据流:
 *   雷达/模拟端 --(网线 UDP, 反无雷达协议帧)--> RadarBridge 监听端口
 *       -> 解码 -> 每个目标编码成 UAV_INFO -> Vehicle::sendMessageOnLinkThreadSafe
 *       -> PX4 handle_message_uav_info -> follower_info
 *       -> target_frame_udp `mavdump on` 打印协议十六进制
 *
 * 说明: 为避免依赖 QGC 尚未装配的 UAV_INFO dialect 头, 本类用通用的
 *       mavlink_finalize_message_chan() 手工构造消息, crc_extra 显式给出。
 *
 * 可通过环境变量配置(可选):
 *   QGC_RADAR_PORT   监听 UDP 端口 (默认 50000)
 *   QGC_RADAR_CRC    协议 CRC 模式: none|modbus|ccitt|x25 (默认 modbus)
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QObject>
#include <QtCore/QLoggingCategory>

#include <cstdint>

class QUdpSocket;

Q_DECLARE_LOGGING_CATEGORY(RadarBridgeLog)

class RadarBridge : public QObject
{
    Q_OBJECT

public:
    explicit RadarBridge(QObject *parent = nullptr);
    ~RadarBridge() override;

    /// 获取单例。
    static RadarBridge *instance();

    /// 开启 UDP 监听。应在 LinkManager/MAVLinkProtocol 初始化之后调用。
    void init();

    enum class CrcMode : uint8_t { None = 0, Modbus, CcittFalse, X25 };

private slots:
    void _readPendingDatagrams();

private:
    struct RadarTarget {
        int32_t  longitude_e7 = 0;
        int32_t  latitude_e7  = 0;
        int32_t  altitude_mm  = 0;
        uint16_t target_id    = 0;
        uint16_t delete_flag  = 0;
        float    speed_m_s    = 0.f;
        float    altitude_m   = 0.f;
        uint8_t  track_type   = 0x20;
        uint8_t  target_attribute = 0x22;
        float    vx_m_s       = 0.f;
        float    vy_m_s       = 0.f;
        float    vz_m_s       = 0.f;
    };

    void _processRadarFrame(const uint8_t *buffer, int length);
    bool _decodeRadarFrame(const uint8_t *buffer, int length, RadarTarget *targets,
                           int targetCapacity, int &targetCount);
    void _sendUavInfo(const RadarTarget &target);
    static uint16_t _crc16(CrcMode mode, const uint8_t *data, int length);

    QUdpSocket *_socket = nullptr;
    quint16     _listenPort = 50000;
    CrcMode     _crcMode = CrcMode::Modbus;
    uint64_t    _rxFrames = 0;
    uint64_t    _txMessages = 0;
    uint64_t    _dropped = 0;
};
