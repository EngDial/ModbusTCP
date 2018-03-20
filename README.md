# ModbusTCP
Library for Arduino for work on UART by the ModbusTCP protocol.

Кадр данных в ModbusTCP отличается от ModbusRtu.
Пакет протокола, так называемый PDU (Protocol Data Unit) у обоих протоколов одинаковый:

PDU = [Function (1 byte)] | [Data (2..252 bytes)]

Для ModbusRtu кадр данных:

[ID (1 byte)] | [PDU (3..253 bytes)] | [CRC16 (2 bytes)]

Для ModbusTCP кадр данных:

[MBAP (7 bytes)] | [PDU (3..253 bytes)]

MBAP Header (Modbus Application Header):

[IDTRAN (2 bytes)] | [IDPROT (2 bytes)] | [LENGTH (2 bytes)] | [UID (1 byte)]

IDTRAN - уникальный ID Transaction (не изменяется при ответе)
IDPROT - ID протокола (всегда 0 - для дальнейшего использования)
LENGTH - Длина следующих данных: PDU + 1 (UID) (изменяется в зависимости от PDU)
UID - адрес ведомого устройства, = ID в ModbusRtu
