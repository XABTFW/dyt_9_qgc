/****************************************************************************
 *
 * TCS virtual serial port -> DYT track-point MAVLink bridge.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QElapsedTimer>
#include <QtCore/QLoggingCategory>
#include <QtCore/QObject>
#include <QtCore/QStringList>

class QSerialPort;
class QTimer;

Q_DECLARE_LOGGING_CATEGORY(DytSerialBridgeLog)

class DytSerialBridge : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QStringList availablePorts READ availablePorts NOTIFY availablePortsChanged)
    Q_PROPERTY(QString portName READ portName NOTIFY connectionChanged)
    Q_PROPERTY(bool connected READ connected NOTIFY connectionChanged)
    Q_PROPERTY(QString statusText READ statusText NOTIFY statusTextChanged)
    Q_PROPERTY(QString lastPointText READ lastPointText NOTIFY lastPointChanged)
    Q_PROPERTY(int imageWidth READ imageWidth NOTIFY configurationChanged)
    Q_PROPERTY(int imageHeight READ imageHeight NOTIFY configurationChanged)

public:
    explicit DytSerialBridge(QObject *parent = nullptr);
    ~DytSerialBridge() override;

    static DytSerialBridge *instance();

    void init();

    QStringList availablePorts() const { return _availablePorts; }
    QString portName() const { return _portName; }
    bool connected() const;
    QString statusText() const { return _statusText; }
    QString lastPointText() const;
    int imageWidth() const { return _imageWidth; }
    int imageHeight() const { return _imageHeight; }

    Q_INVOKABLE void refreshPorts();
    Q_INVOKABLE bool openPort(const QString &portName, int imageWidth, int imageHeight);
    Q_INVOKABLE void closePort();
    Q_INVOKABLE bool resendLastPoint();

signals:
    void availablePortsChanged();
    void connectionChanged();
    void statusTextChanged();
    void lastPointChanged();
    void configurationChanged();

private slots:
    void _readAvailableData();
    void _sendSyntheticStatusFrame();

private:
    void _processFrame(const QByteArray &frame);
    bool _forwardTrackPoint(int centeredX, int centeredY);
    void _setStatusText(const QString &statusText);
    void _closePort(bool updateStatus);

    QSerialPort *_serialPort = nullptr;
    QTimer *_statusTimer = nullptr;
    QByteArray _rxBuffer;
    QStringList _availablePorts;
    QString _portName;
    QString _statusText = tr("Not connected");
    int _imageWidth = 1920;
    int _imageHeight = 1080;
    int _lastX = 0;
    int _lastY = 0;
    int _pendingX = 0;
    int _pendingY = 0;
    bool _hasLastPoint = false;
    bool _hasPendingPoint = false;
    quint64 _invalidFrameCount = 0;
    QElapsedTimer _pendingPointTimer;
    QElapsedTimer _tcsActivityTimer;
};
