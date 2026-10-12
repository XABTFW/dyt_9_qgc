/****************************************************************************
 *
 * RadarBridge 实现: 反无雷达协议帧(UDP) -> MAVLink UAV_INFO -> 飞控
 *
 ****************************************************************************/

#include "RadarBridge.h"

#include "MAVLinkProtocol.h"
#include "MAVLinkLib.h"
#include "MultiVehicleManager.h"
#include "Vehicle.h"
#include "VehicleLinkManager.h"
#include "LinkInterface.h"
#include "QGCLoggingCategory.h"

#include <QtCore/QByteArray>
#include <QtCore/QString>
#include <QtCore/qapplicationstatic.h>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QUdpSocket>

#include <cstring>
#include <cmath>

QGC_LOGGING_CATEGORY(RadarBridgeLog, "qgc.radarbridge")

// ---- 反无雷达协议常量 (与 PX4 target_frame_codec 一致) ----
namespace {
constexpr uint8_t  FRAME_HEADER         = 0xFD;
constexpr uint8_t  PAYLOAD_LENGTH_FIELD = 0x10;
constexpr uint8_t  MESSAGE_TYPE         = 0x01;
constexpr uint8_t  RESERVED             = 0x01;
constexpr int      HEADER_SIZE          = 19; // 8-byte (u64) time field
constexpr int      TARGET_SIZE          = 50;
constexpr int      CRC_SIZE             = 2;
constexpr int      MAX_DATAGRAM_SIZE    = 1472;
constexpr int      MAX_TARGETS          = (MAX_DATAGRAM_SIZE - HEADER_SIZE - CRC_SIZE) / TARGET_SIZE;

// ---- UAV_INFO (common.xml id=12921, mavgen crc_extra=100) ----
constexpr uint32_t UAV_INFO_MSG_ID    = 12921;
constexpr uint8_t  UAV_INFO_CRC_EXTRA = 100;
constexpr uint8_t  UAV_INFO_LEN       = 45;

inline uint16_t get_u16(const uint8_t *b, int off)
{
    return static_cast<uint16_t>(b[off]) | static_cast<uint16_t>(b[off + 1] << 8);
}
inline uint32_t get_u32(const uint8_t *b, int off)
{
    return static_cast<uint32_t>(b[off]) | (static_cast<uint32_t>(b[off + 1]) << 8) |
           (static_cast<uint32_t>(b[off + 2]) << 16) | (static_cast<uint32_t>(b[off + 3]) << 24);
}
inline float get_float(const uint8_t *b, int off)
{
    uint32_t bits = get_u32(b, off);
    float v = 0.f;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}
inline int frame_size(int count) { return HEADER_SIZE + count * TARGET_SIZE + CRC_SIZE; }
} // namespace

Q_APPLICATION_STATIC(RadarBridge, _radarBridgeInstance);

RadarBridge::RadarBridge(QObject *parent)
    : QObject(parent)
{
}

RadarBridge::~RadarBridge()
{
}

RadarBridge *RadarBridge::instance()
{
    return _radarBridgeInstance();
}

void RadarBridge::init()
{
    // 环境变量可覆盖默认监听端口与 CRC 模式
    const QByteArray portEnv = qgetenv("QGC_RADAR_PORT");
    if (!portEnv.isEmpty()) {
        bool ok = false;
        const uint p = portEnv.toUInt(&ok);
        if (ok && p > 0 && p <= 65535) {
            _listenPort = static_cast<quint16>(p);
        }
    }

    const QByteArray crcEnv = qgetenv("QGC_RADAR_CRC");
    if (!crcEnv.isEmpty()) {
        const QString c = QString::fromLatin1(crcEnv).trimmed().toLower();
        if (c == "none")        { _crcMode = CrcMode::None; }
        else if (c == "modbus") { _crcMode = CrcMode::Modbus; }
        else if (c == "ccitt" || c == "ccitt-false") { _crcMode = CrcMode::CcittFalse; }
        else if (c == "x25")    { _crcMode = CrcMode::X25; }
    }

    if (_socket) {
        return;
    }

    _socket = new QUdpSocket(this);
    if (!_socket->bind(QHostAddress::AnyIPv4, _listenPort, QUdpSocket::ShareAddress)) {
        qCWarning(RadarBridgeLog) << "绑定 UDP 端口失败" << _listenPort << _socket->errorString();
        delete _socket;
        _socket = nullptr;
        return;
    }

    connect(_socket, &QUdpSocket::readyRead, this, &RadarBridge::_readPendingDatagrams);
    qCInfo(RadarBridgeLog) << "RadarBridge 监听雷达协议帧 UDP 端口" << _listenPort
                           << "CRC=" << static_cast<int>(_crcMode);
}

void RadarBridge::_readPendingDatagrams()
{
    while (_socket && _socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(_socket->pendingDatagramSize()));
        const qint64 n = _socket->readDatagram(datagram.data(), datagram.size());
        if (n <= 0) {
            continue;
        }
        _processRadarFrame(reinterpret_cast<const uint8_t *>(datagram.constData()),
                           static_cast<int>(n));
    }
}

void RadarBridge::_processRadarFrame(const uint8_t *buffer, int length)
{
    RadarTarget targets[MAX_TARGETS];
    int count = 0;

    if (!_decodeRadarFrame(buffer, length, targets, MAX_TARGETS, count)) {
        ++_dropped;
        if ((_dropped % 100) == 1) {
            qCWarning(RadarBridgeLog) << "丢弃无效雷达帧, 累计" << _dropped;
        }
        return;
    }

    ++_rxFrames;
    qCDebug(RadarBridgeLog) << "收到雷达帧, 目标数" << count << "rx_frames" << _rxFrames;

    for (int i = 0; i < count; ++i) {
        if (targets[i].delete_flag != 0) {
            continue;
        }
        _sendUavInfo(targets[i]);
    }
}

bool RadarBridge::_decodeRadarFrame(const uint8_t *buffer, int length, RadarTarget *targets,
                                    int targetCapacity, int &targetCount)
{
    targetCount = 0;
    if (!buffer || length < frame_size(1)) { return false; }
    if (buffer[0] != FRAME_HEADER || buffer[3] != MESSAGE_TYPE || buffer[4] != RESERVED) { return false; }
    if (buffer[5] != 0xC5 || buffer[6] != 0xCE || buffer[7] != 0xC2) { return false; }
    if (buffer[1] != PAYLOAD_LENGTH_FIELD) { return false; }

    const int count = buffer[18];
    if (count == 0 || count > MAX_TARGETS || count > targetCapacity) { return false; }

    const int expected = frame_size(count);
    if (length != expected || static_cast<int>(get_u16(buffer, 16)) != expected) { return false; }

    if (_crcMode != CrcMode::None) {
        const uint16_t got = get_u16(buffer, length - CRC_SIZE);
        // CRC covers payload length through payload data. Exclude the 0xFD frame header and CRC field itself.
        const uint16_t calc = _crc16(_crcMode, buffer + 1, length - CRC_SIZE - 1);
        if (got != calc) { return false; }
    }

    for (int i = 0; i < count; ++i) {
        const uint8_t *t = &buffer[HEADER_SIZE + i * TARGET_SIZE];
        RadarTarget &out = targets[i];
        out.longitude_e7     = static_cast<int32_t>(get_u32(t, 0));
        out.latitude_e7      = static_cast<int32_t>(get_u32(t, 4));
        out.altitude_mm      = static_cast<int32_t>(get_u32(t, 8));
        out.target_id        = get_u16(t, 12);
        out.delete_flag      = get_u16(t, 14);
        out.speed_m_s        = get_float(t, 16);
        out.altitude_m       = get_float(t, 32);
        out.track_type       = t[36];
        out.target_attribute = t[37];
        out.vx_m_s           = get_float(t, 38);
        out.vy_m_s           = get_float(t, 42);
        out.vz_m_s           = get_float(t, 46);
    }
    targetCount = count;
    return true;
}

void RadarBridge::_sendUavInfo(const RadarTarget &target)
{
    Vehicle *vehicle = MultiVehicleManager::instance()->activeVehicle();
    if (!vehicle) {
        qCDebug(RadarBridgeLog) << "无 active vehicle, 丢弃目标" << target.target_id;
        return;
    }

    WeakLinkInterfacePtr weakLink = vehicle->vehicleLinkManager()->primaryLink();
    if (weakLink.expired()) {
        return;
    }
    SharedLinkInterfacePtr sharedLink = weakLink.lock();
    if (!sharedLink) {
        return;
    }

    // 组装 UAV_INFO 45 字节载荷 (线序: mavid,group_id,lat,lon,yaw,yaw_speed,
    // rel_alt,vx,vy,vz,land 均在前 44 字节, is_leader 为最后 1 字节)
    const float lat = static_cast<float>(static_cast<double>(target.latitude_e7) * 1e-7);
    const float lon = static_cast<float>(static_cast<double>(target.longitude_e7) * 1e-7);

    char buf[UAV_INFO_LEN];
    _mav_put_uint32_t(buf, 0,  target.target_id); // mavid = 目标编号
    _mav_put_uint32_t(buf, 4,  0);                // group_id
    _mav_put_float(buf, 8,  lat);
    _mav_put_float(buf, 12, lon);
    _mav_put_float(buf, 16, 0.f);                 // yaw
    _mav_put_float(buf, 20, 0.f);                 // yaw_speed
    _mav_put_float(buf, 24, target.altitude_m);   // rel_alt
    _mav_put_float(buf, 28, target.vx_m_s);
    _mav_put_float(buf, 32, target.vy_m_s);
    _mav_put_float(buf, 36, target.vz_m_s);
    _mav_put_uint32_t(buf, 40, 0);                // land
    _mav_put_uint8_t(buf, 44, 0);                 // is_leader=0 -> PX4 落到 follower_info.mavid=目标编号

    mavlink_message_t msg;
    std::memset(&msg, 0, sizeof(msg));
    std::memcpy(_MAV_PAYLOAD_NON_CONST(&msg), buf, UAV_INFO_LEN);
    msg.msgid = UAV_INFO_MSG_ID;

    const uint8_t sysid  = static_cast<uint8_t>(MAVLinkProtocol::instance()->getSystemId());
    const uint8_t compid = static_cast<uint8_t>(MAVLinkProtocol::getComponentId());

    mavlink_finalize_message_chan(&msg, sysid, compid, sharedLink->mavlinkChannel(),
                                  UAV_INFO_LEN, UAV_INFO_LEN, UAV_INFO_CRC_EXTRA);

    if (vehicle->sendMessageOnLinkThreadSafe(sharedLink.get(), msg)) {
        ++_txMessages;
        const double speed = std::sqrt(static_cast<double>(target.vx_m_s) * target.vx_m_s +
                                       static_cast<double>(target.vy_m_s) * target.vy_m_s +
                                       static_cast<double>(target.vz_m_s) * target.vz_m_s);
        qCDebug(RadarBridgeLog).nospace()
            << "-> UAV_INFO id=" << target.target_id
            << " lon=" << lon << " lat=" << lat
            << " alt=" << target.altitude_m << " speed=" << speed
            << " (tx=" << _txMessages << ")";
    }
}

uint16_t RadarBridge::_crc16(CrcMode mode, const uint8_t *data, int length)
{
    switch (mode) {
    case CrcMode::None:
        return 0;

    case CrcMode::Modbus: {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < length; ++i) {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit) {
                crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                                : static_cast<uint16_t>(crc >> 1);
            }
        }
        return crc;
    }

    case CrcMode::CcittFalse: {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i] << 8);
            for (int bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                     : static_cast<uint16_t>(crc << 1);
            }
        }
        return crc;
    }

    case CrcMode::X25: {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < length; ++i) {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit) {
                crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408)
                                : static_cast<uint16_t>(crc >> 1);
            }
        }
        return static_cast<uint16_t>(crc ^ 0xFFFF);
    }
    }
    return 0;
}
