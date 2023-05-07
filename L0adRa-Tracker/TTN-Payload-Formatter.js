function decodeUplink(input) {
  return {
    data: Decoder(input.bytes, input.fPort),
    warnings: [],
    errors: []
  };
}

// Decode a Little-Endian serialized float value
// see https://gist.github.com/kg/2192799
function decodeFloat(bytes, offset) {
 var totalBits = 32;

 var binary = "";
 for (var i = 0, l = 4; i < l; i++) {
   var bits = bytes[offset+i].toString(2);
   while (bits.length < 8)
     bits = "0" + bits;

   binary = bits + binary;
 }

 var sign = (binary.charAt(0) == '1')?-1:1;
 var exponent = parseInt(binary.substr(1, 8), 2) - 127;
 var significandBase = binary.substr(1 + 8, 23);
 var significandBin = '1'+significandBase;
 var i = 0;
 var val = 1;
 var significand = 0;

 if (exponent == -127) {
     if (significandBase.indexOf('1') == -1)
         return 0;
     else {
         exponent = -126;
         significandBin = '0'+significandBase;
     }
 }

 while (i < significandBin.length) {
     significand += val * parseInt(significandBin.charAt(i));
     val = val / 2;
     i++;
 }

 return sign * significand * Math.pow(2, exponent);
}

// Decode a bit flag
function decodeBit(bytes, byteOffset, bitOffset) {
 return (bytes[byteOffset] & (0b00000001 << bitOffset)) > 0;
}

function decodeByte(bytes, offset) {
 return bytes[offset];
}

// Decode a Little-Endian serialized short value
function decodeShort(bytes, offset) {
 return 256*bytes[offset+1] + bytes[offset];
}

// Decode a Little-Endian serialized int value
function decodeInt(bytes, offset) {
 var result = bytes[offset+3];
 result = result*256 + bytes[offset+2];
 result = result*256 + bytes[offset+1];
 result = result*256 + bytes[offset];
 return result;
}

// Decode an array of bytes into an JSON object.
//  - fPort contains the LoRaWAN fPort number
//  - bytes is an array of bytes, e.g. [225, 230, 255, 0]
//  - variables contains the device variables e.g. {"calibration": "3.5"} (both the key / value are of type string)
// function must return an object, e.g. {"temperature": 22.5}

function Decoder(bytes, port) {

  const SYSTIME_PAYLOAD_FLAG_OFFSET = 0;
  const SYSTIME_PAYLOAD_FLAG_BIT = 7;

  const DHT_TEMP_PAYLOAD_FLAG_OFFSET = 0;
  const DHT_TEMP_PAYLOAD_FLAG_BIT = 6;
  const DHT_TEMP_MIN_VALUE = -74.0;
  const DHT_TEMP_ACCURACY = 0.5;

  const DHT_HUMIDITY_PAYLOAD_FLAG_OFFSET = 0;
  const DHT_HUMIDITY_PAYLOAD_FLAG_BIT = 5;
  const DHT_HUMIDITY_MIN_VALUE = 0.0;
  const DHT_HUMIDITY_ACCURACY = 1.0;

  const BMP_TEMP_PAYLOAD_FLAG_OFFSET = 0;
  const BMP_TEMP_PAYLOAD_FLAG_BIT = 4;
  const BMP_TEMP_MIN_VALUE = -74.0;
  const BMP_TEMP_ACCURACY = 0.5;

  const BMP_PRESSURE_PAYLOAD_FLAG_OFFSET = 0;
  const BMP_PRESSURE_PAYLOAD_FLAG_BIT = 3;
  const BMP_PRESSURE_MIN_VALUE = 0.0;
  const BMP_PRESSURE_ACCURACY = 2.0;

  const SCD_TEMP_PAYLOAD_FLAG_OFFSET = 0;
  const SCD_TEMP_PAYLOAD_FLAG_BIT = 2;
  const SCD_TEMP_MIN_VALUE = -74.0;
  const SCD_TEMP_ACCURACY = 0.5;

  const SCD_HUMIDITY_PAYLOAD_FLAG_OFFSET = 0;
  const SCD_HUMIDITY_PAYLOAD_FLAG_BIT = 1;
  const SCD_HUMIDITY_MIN_VALUE = 0.0;
  const SCD_HUMIDITY_ACCURACY = 1.0;

  const SCD_CO2_PAYLOAD_FLAG_OFFSET = 0;
  const SCD_CO2_PAYLOAD_FLAG_BIT = 0;
  const SCD_CO2_MIN_VALUE = 0.0;
  const SCD_CO2_ACCURACY = 10;

  const VBAT_PAYLOAD_FLAG_OFFSET = 1;
  const VBAT_PAYLOAD_FLAG_BIT = 7;
  const VBAT_MIN_VALUE = 0.0;
  const VBAT_ACCURACY = 0.02;

  const GPS_PAYLOAD_FLAG_OFFSET = 1;
  const GPS_PAYLOAD_FLAG_BIT = 6;
  const GPS_ALTITUDE_MIN_VALUE = 0.0;
  const GPS_ALTITUDE_ACCURACY = 1.0;

  result = {}
  var offset = 2;
  //if (decodeBit(bytes,SYSTIME_PAYLOAD_FLAG_OFFSET, SYSTIME_PAYLOAD_FLAG_BIT) == 1) {
    var sysTimeValue = decodeShort(bytes, offset);
    result["sys_time"] = parseInt(sysTimeValue.toFixed(0));
    offset += 2;
  //}
  if (decodeBit(bytes, DHT_TEMP_PAYLOAD_FLAG_OFFSET, DHT_TEMP_PAYLOAD_FLAG_BIT) == 1) {
    var dhtTempValue = (decodeByte(bytes, offset) * DHT_TEMP_ACCURACY) + DHT_TEMP_MIN_VALUE;
    result["dht_temp"] = parseFloat(dhtTempValue.toFixed(1));
    offset += 1;
  }
  if (decodeBit(bytes,DHT_HUMIDITY_PAYLOAD_FLAG_OFFSET, DHT_HUMIDITY_PAYLOAD_FLAG_BIT) == 1) {
    var dhtHumidityValue = (decodeByte(bytes, offset) * DHT_HUMIDITY_ACCURACY) + DHT_HUMIDITY_MIN_VALUE;
    result["dht_humidity"] = parseInt(dhtHumidityValue.toFixed(0));
    offset += 1;
  }
  if (decodeBit(bytes,BMP_TEMP_PAYLOAD_FLAG_OFFSET, BMP_TEMP_PAYLOAD_FLAG_BIT) == 1) {
    var bmpTempValue = (decodeByte(bytes, offset) * BMP_TEMP_ACCURACY) + BMP_TEMP_MIN_VALUE;
    result["bmp_temp"] = parseFloat(bmpTempValue.toFixed(1));
    offset += 1;
  }
  if (decodeBit(bytes,BMP_PRESSURE_PAYLOAD_FLAG_OFFSET, BMP_PRESSURE_PAYLOAD_FLAG_BIT) == 1) {
    var bmpPressureValue = (decodeShort(bytes, offset) * BMP_PRESSURE_ACCURACY) + BMP_PRESSURE_MIN_VALUE;
    result["bmp_pressure"] = parseInt(bmpPressureValue.toFixed(0));
    offset += 2;
  }
  if (decodeBit(bytes,SCD_TEMP_PAYLOAD_FLAG_OFFSET, SCD_TEMP_PAYLOAD_FLAG_BIT) == 1) {
    var scdTempValue = (decodeByte(bytes, offset) * SCD_TEMP_ACCURACY) + SCD_TEMP_MIN_VALUE;
    result["scd_temp"] = parseFloat(scdTempValue.toFixed(1));
    offset += 1;
  }
  if (decodeBit(bytes,SCD_HUMIDITY_PAYLOAD_FLAG_OFFSET, SCD_HUMIDITY_PAYLOAD_FLAG_BIT) == 1) {
    var scdHumidityValue = (decodeByte(bytes, offset) * SCD_HUMIDITY_ACCURACY) + SCD_HUMIDITY_MIN_VALUE;
    result["scd_humidity"] = parseInt(scdHumidityValue.toFixed(0));
    offset += 1;
  }
  if (decodeBit(bytes,SCD_CO2_PAYLOAD_FLAG_OFFSET, SCD_CO2_PAYLOAD_FLAG_BIT) == 1) {
    var scdCo2Value = (decodeByte(bytes, offset) * SCD_CO2_ACCURACY) + SCD_CO2_MIN_VALUE;
    result["scd_co2"] = parseInt(scdCo2Value.toFixed(0));
    offset += 1;
  }
  if (decodeBit(bytes,VBAT_PAYLOAD_FLAG_OFFSET, VBAT_PAYLOAD_FLAG_BIT) == 1) {
    var vbatValue = (decodeByte(bytes, offset) * VBAT_ACCURACY) + VBAT_MIN_VALUE;
    result["vbat"] = parseInt(vbatValue.toFixed(0));
    offset += 1;
  }
  result["gps_fix"] = 0;
  if (decodeBit(bytes,GPS_PAYLOAD_FLAG_OFFSET, GPS_PAYLOAD_FLAG_BIT) == 1) {
    result["gps_fix"] = 1;
    result["gps_time"] = decodeInt(bytes, offset);
    offset += 4;
    var gpsAltitudeValue = (decodeShort(bytes, offset) * GPS_ALTITUDE_ACCURACY) + GPS_ALTITUDE_MIN_VALUE;
    result["gps_altitude"] = parseInt(gpsAltitudeValue.toFixed(0));
    offset += 2;
    result["latitude"] = parseFloat(decodeFloat(bytes, offset).toFixed(4));
    offset += 4;
    result["longitude"] = parseFloat(decodeFloat(bytes, offset).toFixed(4));
    offset += 4;
  }

    return result;
}
