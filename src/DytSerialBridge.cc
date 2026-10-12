/****************************************************************************
 *
 * TCS virtual serial port -> DYT track-point MAVLink bridge.
 *
 ****************************************************************************/

#include "DytSerialBridge.h"

#include "MultiVehicleManager.h"
#include "QGCLoggingCategory.h"
#include "Vehicle.h"

#include <QtCore/qapplicationstatic.h>
#include <QtCore/QDateTime>
#include <QtCore/QFile>
#include <QtCore/QTimer>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <algorithm>

QGC_LOGGING_CATEGORY(DytSerialBridgeLog, "qgc.dytserialbridge")

namespace {
constexpr int TcsControlFrameSize = 33;
constexpr quint8 TcsControlPayloadLength = 0x1D;
constexpr quint8 TcsFrameId = 0x00;
constexpr quint8 TcsTrackMode = 0x06;
constexpr quint8 TcsNormalTrackSelection = 0x00;
constexpr int TcsSerialRequestFrameSize = 46;
constexpr quint8 TcsSerialRequestPayloadLength = 0x2A;
constexpr quint8 TcsDebugFrameId = 0x05;
constexpr quint8 TcsSerialRequestControl = 0xFF;
constexpr int TcsServoStatusFrameSize = 45;
constexpr quint8 TcsServoStatusPayloadLength = 0x29;
constexpr int TcsServoStatusIntervalMs = 20;
constexpr int TcsActivityTimeoutMs = 2500;
constexpr int DoubleClickWindowMs = 500;
constexpr int DoubleClickCoordinateTolerancePx = 12;

quint8 byteAt(const QByteArray &data, int index)
{
    return static_cast<quint8>(data.at(index));
}

void traceBridgeEvent(const QString &message)
{
    const QString tracePath = qEnvironmentVariable("QGC_TCS_TRACE_FILE").trimmed();

    if (tracePath.isEmpty()) {
        return;
    }

    QFile traceFile(tracePath);

    if (traceFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        traceFile.write(QDateTime::currentDateTime().toString(Qt::ISODateWithMs).toUtf8());
        traceFile.write(" ");
        traceFile.write(message.toUtf8());
        traceFile.write("\n");
    }
}
}

Q_APPLICATION_STATIC(DytSerialBridge, _dytSerialBridgeInstance);

DytSerialBridge::DytSerialBridge(QObject *parent)
    : QObject(parent)
    , _serialPort(new QSerialPort(this))
    , _statusTimer(new QTimer(this))
{
    _statusTimer->setTimerType(Qt::PreciseTimer);
    _statusTimer->setInterval(TcsServoStatusIntervalMs);
    connect(_statusTimer, &QTimer::timeout, this, &DytSerialBridge::_sendSyntheticStatusFrame);
    connect(_serialPort, &QSerialPort::readyRead, this, &DytSerialBridge::_readAvailableData);
    connect(_serialPort, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error == QSerialPort::NoError) {
            return;
        }

        const QString errorText = tr("Serial error on %1: %2").arg(_portName, _serialPort->errorString());
        qCWarning(DytSerialBridgeLog) << errorText;

        if (error == QSerialPort::ResourceError ||
            error == QSerialPort::DeviceNotFoundError ||
            error == QSerialPort::PermissionError) {
            _closePort(false);
        }

        _setStatusText(errorText);
    });
}

DytSerialBridge::~DytSerialBridge()
{
    _closePort(false);
}

DytSerialBridge *DytSerialBridge::instance()
{
    return _dytSerialBridgeInstance();
}

void DytSerialBridge::init()
{
    refreshPorts();

    const QByteArray autoConnectValue = qgetenv("QGC_TCS_AUTO_CONNECT").trimmed().toLower();

    if (autoConnectValue == "0" || autoConnectValue == "false" || autoConnectValue == "no") {
        return;
    }

    const QString configuredPort = QString::fromLocal8Bit(qgetenv("QGC_TCS_PORT")).trimmed();
    const QString autoPort = configuredPort.isEmpty() ? QStringLiteral("COM21") : configuredPort;
    bool widthOk = false;
    bool heightOk = false;
    const int configuredWidth = QString::fromLocal8Bit(qgetenv("QGC_TCS_IMAGE_WIDTH")).toInt(&widthOk);
    const int configuredHeight = QString::fromLocal8Bit(qgetenv("QGC_TCS_IMAGE_HEIGHT")).toInt(&heightOk);

    if (_availablePorts.contains(autoPort, Qt::CaseInsensitive)) {
        openPort(autoPort,
                 widthOk ? configuredWidth : _imageWidth,
                 heightOk ? configuredHeight : _imageHeight);
    } else {
        _setStatusText(tr("TCS auto-connect port %1 is not available.").arg(autoPort));
    }
}

bool DytSerialBridge::connected() const
{
    return _serialPort && _serialPort->isOpen();
}

QString DytSerialBridge::lastPointText() const
{
    return _hasLastPoint ? tr("(%1, %2)").arg(_lastX).arg(_lastY) : tr("No target received");
}

void DytSerialBridge::refreshPorts()
{
    QStringList ports;

    for (const QSerialPortInfo &portInfo : QSerialPortInfo::availablePorts()) {
        ports.append(portInfo.portName());
    }

    ports.removeDuplicates();
    std::sort(ports.begin(), ports.end(), [](const QString &lhs, const QString &rhs) {
        return QString::localeAwareCompare(lhs, rhs) < 0;
    });

    if (ports != _availablePorts) {
        _availablePorts = ports;
        emit availablePortsChanged();
    }
}

bool DytSerialBridge::openPort(const QString &portName, int imageWidth, int imageHeight)
{
    const QString trimmedPortName = portName.trimmed();

    if (trimmedPortName.isEmpty()) {
        _setStatusText(tr("Select the QGC side of the virtual serial-port pair."));
        return false;
    }

    if (imageWidth <= 0 || imageWidth > 65535 || imageHeight <= 0 || imageHeight > 65535) {
        _setStatusText(tr("Image width and height must be in the range 1..65535."));
        return false;
    }

    _closePort(false);
    _portName = trimmedPortName;
    _imageWidth = imageWidth;
    _imageHeight = imageHeight;
    _rxBuffer.clear();
    _hasPendingPoint = false;
    _statusTimer->stop();
    emit configurationChanged();

    _serialPort->setPortName(_portName);
    _serialPort->setBaudRate(QSerialPort::Baud115200);
    _serialPort->setDataBits(QSerialPort::Data8);
    _serialPort->setParity(QSerialPort::NoParity);
    _serialPort->setStopBits(QSerialPort::OneStop);
    _serialPort->setFlowControl(QSerialPort::NoFlowControl);
    _serialPort->setReadBufferSize(4096);

    if (!_serialPort->open(QIODevice::ReadWrite)) {
        _setStatusText(tr("Cannot open %1: %2").arg(_portName, _serialPort->errorString()));
        emit connectionChanged();
        return false;
    }

    qCInfo(DytSerialBridgeLog) << "Listening for TCS control frames on" << _portName
                               << "at 115200 8N1, image" << _imageWidth << "x" << _imageHeight;
    traceBridgeEvent(QStringLiteral("listening port=%1 image=%2x%3").arg(_portName).arg(_imageWidth).arg(_imageHeight));
    _setStatusText(tr("Listening on %1 at 115200 8N1").arg(_portName));
    emit connectionChanged();
    return true;
}

void DytSerialBridge::closePort()
{
    _closePort(true);
}

void DytSerialBridge::_closePort(bool updateStatus)
{
    const bool wasConnected = connected();

    if (_serialPort && _serialPort->isOpen()) {
        _serialPort->close();
    }

    _rxBuffer.clear();
    _hasPendingPoint = false;
    _statusTimer->stop();

    if (wasConnected) {
        emit connectionChanged();
    }

    if (updateStatus) {
        _setStatusText(tr("Not connected"));
    }
}

bool DytSerialBridge::resendLastPoint()
{
    if (!_hasLastPoint) {
        _setStatusText(tr("No valid TCS target has been received."));
        return false;
    }

    return _forwardTrackPoint(_lastX, _lastY);
}

void DytSerialBridge::_readAvailableData()
{
    _rxBuffer.append(_serialPort->readAll());
    const QByteArray header("\x55\xAA", 2);

    while (_rxBuffer.size() >= 3) {
        const qsizetype headerOffset = _rxBuffer.indexOf(header);

        if (headerOffset < 0) {
            const bool keepPossibleHeader = byteAt(_rxBuffer, _rxBuffer.size() - 1) == 0x55;
            _rxBuffer = keepPossibleHeader ? QByteArray(1, static_cast<char>(0x55)) : QByteArray();
            return;
        }

        if (headerOffset > 0) {
            _rxBuffer.remove(0, headerOffset);
        }

        if (_rxBuffer.size() < 3) {
            return;
        }

        const int frameSize = static_cast<int>(byteAt(_rxBuffer, 2)) + 4;

        if (frameSize < 5) {
            _rxBuffer.remove(0, 1);
            continue;
        }

        if (_rxBuffer.size() < frameSize) {
            return;
        }

        quint8 checksum = 0xFF;

        for (int index = 2; index < frameSize - 1; ++index) {
            checksum ^= byteAt(_rxBuffer, index);
        }

        if (checksum != byteAt(_rxBuffer, frameSize - 1)) {
            ++_invalidFrameCount;

            if (_invalidFrameCount == 1 || (_invalidFrameCount % 100) == 0) {
                qCWarning(DytSerialBridgeLog) << "Dropping TCS frame with invalid XOR checksum, count"
                                               << _invalidFrameCount;
            }

            _rxBuffer.remove(0, 1);
            continue;
        }

        const QByteArray frame = _rxBuffer.left(frameSize);
        _rxBuffer.remove(0, frameSize);
        _processFrame(frame);
    }
}

void DytSerialBridge::_processFrame(const QByteArray &frame)
{
    _tcsActivityTimer.restart();

    if (frame.size() == TcsSerialRequestFrameSize &&
        byteAt(frame, 2) == TcsSerialRequestPayloadLength &&
        byteAt(frame, 3) == TcsDebugFrameId &&
        byteAt(frame, 4) == TcsSerialRequestControl) {
        const qint64 written = _serialPort->write(frame);

        if (written == frame.size()) {
            traceBridgeEvent(QStringLiteral("serial-request acknowledged bytes=%1").arg(written));
        } else {
            traceBridgeEvent(QStringLiteral("serial-request acknowledge failed written=%1 error=%2")
                                 .arg(written)
                                 .arg(_serialPort->errorString()));
        }

        _sendSyntheticStatusFrame();
        _statusTimer->start();

        return;
    }

    if (frame.size() != TcsControlFrameSize ||
        byteAt(frame, 2) != TcsControlPayloadLength ||
        byteAt(frame, 3) != TcsFrameId ||
        byteAt(frame, 4) != TcsTrackMode ||
        byteAt(frame, 11) != TcsNormalTrackSelection) {
        return;
    }

    const qint16 x = static_cast<qint16>((static_cast<quint16>(byteAt(frame, 12)) << 8) | byteAt(frame, 13));
    const qint16 y = static_cast<qint16>((static_cast<quint16>(byteAt(frame, 14)) << 8) | byteAt(frame, 15));

    const int absoluteX = (_imageWidth / 2) + x;
    const int absoluteY = (_imageHeight / 2) + y;

    if (absoluteX < 0 || absoluteY < 0 || absoluteX >= _imageWidth || absoluteY >= _imageHeight) {
        _setStatusText(tr("Rejected out-of-range centered TCS target (%1, %2) for %3 x %4 image")
                           .arg(x).arg(y).arg(_imageWidth).arg(_imageHeight));
        return;
    }

    if (!_hasLastPoint || _lastX != x || _lastY != y) {
        _lastX = x;
        _lastY = y;
        _hasLastPoint = true;
        emit lastPointChanged();
    }

    const bool matchingSecondClick = _hasPendingPoint &&
                                     _pendingPointTimer.isValid() &&
                                     _pendingPointTimer.elapsed() <= DoubleClickWindowMs &&
                                     std::abs(_pendingX - static_cast<int>(x)) <= DoubleClickCoordinateTolerancePx &&
                                     std::abs(_pendingY - static_cast<int>(y)) <= DoubleClickCoordinateTolerancePx;

    if (matchingSecondClick) {
        _hasPendingPoint = false;
        traceBridgeEvent(QStringLiteral("double-click centered=(%1,%2)").arg(x).arg(y));
        _forwardTrackPoint(x, y);
        return;
    }

    _pendingX = x;
    _pendingY = y;
    _hasPendingPoint = true;
    _pendingPointTimer.restart();
    traceBridgeEvent(QStringLiteral("first-click centered=(%1,%2)").arg(x).arg(y));
    _setStatusText(tr("Target (%1, %2) received; waiting for double-click confirmation.").arg(x).arg(y));
}

void DytSerialBridge::_sendSyntheticStatusFrame()
{
    if (!connected() || !_tcsActivityTimer.isValid() || _tcsActivityTimer.elapsed() > TcsActivityTimeoutMs) {
        _statusTimer->stop();
        traceBridgeEvent(QStringLiteral("synthetic servo status stopped"));
        return;
    }

    QByteArray statusFrame(TcsServoStatusFrameSize, '\0');
    statusFrame[0] = static_cast<char>(0x55);
    statusFrame[1] = static_cast<char>(0xAA);
    statusFrame[2] = static_cast<char>(TcsServoStatusPayloadLength);
    statusFrame[3] = static_cast<char>(TcsFrameId);
    statusFrame[4] = static_cast<char>(0x00); // Self-test normal.
    statusFrame[5] = static_cast<char>(0x01); // Visible-light image source.
    statusFrame[6] = static_cast<char>(0x01); // Manual servo mode.

    quint8 checksum = 0xFF;

    for (int index = 2; index < statusFrame.size() - 1; ++index) {
        checksum ^= byteAt(statusFrame, index);
    }

    statusFrame[statusFrame.size() - 1] = static_cast<char>(checksum);

    if (_serialPort->write(statusFrame) != statusFrame.size()) {
        traceBridgeEvent(QStringLiteral("synthetic servo status write failed error=%1")
                             .arg(_serialPort->errorString()));
    }
}

bool DytSerialBridge::_forwardTrackPoint(int centeredX, int centeredY)
{
    const int x = (_imageWidth / 2) + centeredX;
    const int y = (_imageHeight / 2) + centeredY;

    if (x < 0 || y < 0 || x >= _imageWidth || y >= _imageHeight) {
        _setStatusText(tr("Centered target (%1, %2) is outside the configured image.").arg(centeredX).arg(centeredY));
        return false;
    }

    Vehicle *const vehicle = MultiVehicleManager::instance()->activeVehicle();

    if (!vehicle) {
        traceBridgeEvent(QStringLiteral("drop absolute=(%1,%2) reason=no-active-vehicle").arg(x).arg(y));
        _setStatusText(tr("Target (%1, %2) received, but no active vehicle is connected.").arg(x).arg(y));
        return false;
    }

    if (!vehicle->sendDytTrackPointCommand(x, y, _imageWidth, _imageHeight)) {
        traceBridgeEvent(QStringLiteral("drop absolute=(%1,%2) reason=vehicle-send-failed").arg(x).arg(y));
        _setStatusText(tr("Target (%1, %2) was not sent to the vehicle.").arg(x).arg(y));
        return false;
    }

    _setStatusText(tr("Target (%1, %2) forwarded to vehicle %3.").arg(x).arg(y).arg(vehicle->id()));
    traceBridgeEvent(QStringLiteral("forward absolute=(%1,%2) vehicle=%3").arg(x).arg(y).arg(vehicle->id()));
    return true;
}

void DytSerialBridge::_setStatusText(const QString &statusText)
{
    if (_statusText == statusText) {
        return;
    }

    _statusText = statusText;
    emit statusTextChanged();
}
