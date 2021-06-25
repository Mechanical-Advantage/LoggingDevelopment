// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging.file;

import java.io.UnsupportedEncodingException;
import java.nio.ByteBuffer;

import frc.robot.logging.core.LogTable.LogValue;

/** Converts log tables to byte array format. */
public class ByteEncoder {

  public static ByteBuffer encodeTimestamp(double timestamp) {
    ByteBuffer buffer = ByteBuffer.allocate(1 + Double.BYTES);
    buffer.put((byte) 0);
    buffer.putDouble(timestamp);
    return buffer;
  }

  public static ByteBuffer encodeKey(short keyID, String key) {
    try {
      ByteBuffer buffer = ByteBuffer.allocate(1 + Short.BYTES + Short.BYTES + key.length());
      buffer.put((byte) 1);
      buffer.putShort(keyID);
      buffer.putShort((short) key.length());
      buffer.put(key.getBytes("UTF-8"));
      return buffer;
    } catch (UnsupportedEncodingException e) {
      return ByteBuffer.allocate(0);
    }
  }

  public static ByteBuffer encodeValue(short keyID, LogValue value) {
    try {
      // Generate key and type buffer
      ByteBuffer keyBuffer = ByteBuffer.allocate(1 + Short.BYTES + 1);
      keyBuffer.put((byte) 2);
      keyBuffer.putShort(keyID);
      if (value == null) {
        keyBuffer.put((byte) 0);
      } else {
        keyBuffer.put((byte) (value.type.ordinal() + 1));
      }

      // Generate value buffer
      ByteBuffer valueBuffer;
      if (value == null) {
        valueBuffer = ByteBuffer.allocate(0);
      } else {
        switch (value.type) {
          case Boolean:
            valueBuffer = ByteBuffer.allocate(1).put(value.getBoolean() ? (byte) 1 : (byte) 0);
            break;
          case Byte:
            valueBuffer = ByteBuffer.allocate(1).put(value.getByte());
            break;
          case Integer:
            valueBuffer = ByteBuffer.allocate(Integer.BYTES).putInt(value.getInteger());
            break;
          case Double:
            valueBuffer = ByteBuffer.allocate(Double.BYTES).putDouble(value.getDouble());
            break;
          case String:
            String stringValue = value.getString();
            valueBuffer = ByteBuffer.allocate(Short.BYTES + stringValue.length());
            valueBuffer.putShort((short) stringValue.length());
            valueBuffer.put(stringValue.getBytes("UTF-8"));
            break;
          case BooleanArray:
            boolean[] booleanArray = value.getBooleanArray();
            valueBuffer = ByteBuffer.allocate(Short.BYTES + booleanArray.length);
            valueBuffer.putShort((short) booleanArray.length);
            for (boolean i : booleanArray) {
              valueBuffer.put(i ? (byte) 1 : (byte) 0);
            }
            break;
          case ByteArray:
            byte[] byteArray = value.getByteArray();
            valueBuffer = ByteBuffer.allocate(Short.BYTES + byteArray.length);
            valueBuffer.putShort((short) byteArray.length);
            for (byte i : byteArray) {
              valueBuffer.put(i);
            }
            break;
          case IntegerArray:
            int[] intArray = value.getIntegerArray();
            valueBuffer = ByteBuffer.allocate(Short.BYTES + (intArray.length * Integer.BYTES));
            valueBuffer.putShort((short) intArray.length);
            for (int i : intArray) {
              valueBuffer.putInt(i);
            }
            break;
          case DoubleArray:
            double[] doubleArray = value.getDoubleArray();
            valueBuffer = ByteBuffer.allocate(Short.BYTES + (doubleArray.length * Double.BYTES));
            valueBuffer.putShort((short) doubleArray.length);
            for (double i : doubleArray) {
              valueBuffer.putDouble(i);
            }
            break;
          case StringArray:
            String[] stringArray = value.getStringArray();
            int capacity = 2;
            for (String i : stringArray) {
              capacity += 2 + i.length();
            }
            valueBuffer = ByteBuffer.allocate(capacity);
            valueBuffer.putShort((short) stringArray.length);
            for (String i : stringArray) {
              valueBuffer.putShort((short) i.length());
              valueBuffer.put(i.getBytes("UTF-8"));
            }
            break;
          default:
            valueBuffer = ByteBuffer.allocate(0);
        }
      }

      return ByteBuffer.allocate(keyBuffer.capacity() + valueBuffer.capacity()).put(keyBuffer.array())
          .put(valueBuffer.array());
    } catch (UnsupportedEncodingException e) {
      return ByteBuffer.allocate(0);
    }
  }
}