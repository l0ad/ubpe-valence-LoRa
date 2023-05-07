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

// Decode a Little-Endian serialized short value
function decodeShort(bytes, offset) {
 return 256*bytes[offset+1] + bytes[offset];
}

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
 return {
         "sys_time": decodeShort(bytes,0),
         "dht_temp": parseFloat(decodeFloat(bytes, 2).toFixed(1)),
         "dht_humidity": parseFloat(decodeFloat(bytes, 6).toFixed(0)),
         "bmp_temp": parseFloat(decodeFloat(bytes, 10).toFixed(1)),
         "bmp_pressure": parseInt(decodeInt(bytes, 14).toFixed(1)),
         "scd_temp": parseFloat(decodeFloat(bytes, 18).toFixed(1)),
         "scd_co2": parseInt(decodeInt(bytes, 22).toFixed(1)),
         "scd_humidity": parseFloat(decodeFloat(bytes, 26).toFixed(1)),
         "gps_fix": bytes[30],
         "gps_time": decodeInt(bytes, 31),
         "gps_altitude": parseFloat(decodeFloat(bytes, 35).toFixed(1)),
         "latitude": parseFloat(decodeFloat(bytes, 39).toFixed(4)),
         "longitude": parseFloat(decodeFloat(bytes, 43).toFixed(4)),
         "vbat": parseFloat(decodeFloat(bytes, 47).toFixed(2))

        };
}
