Gateway using WeMos D1 mini and MCP2515 CANbus controller using WiFi and http

The code is slightly biassed toward the Renault brand in the sense that it uses standard (11 bits) addressing, assumes bare frames for addresses under 0x700, ISOTP formatted diagnostic commands for addresses of 0x700 and above, and does NOT adhere to the convention that an ECU anwers on an address which is 8 above the receiving address.

All these constraints / assumptions can be easily changed.
 
WiFi configuration is implemented using Dean Gording code.

http is unencrypted and unsecured.

The http server implements the following primitives:
- /Init Initialises the MCP2515 CANbus controller
- /FreeFrame?f=x.y Request the value of frame ID x in hex. y is a decimal timeout in ms, but it is ignored
- /IsoTpFrame?f=x.y.z Send hex command string z as ISOTP to ID y and expect answer on ID x. Note that if the first nibble of the command string >= 4, 4 is substracted. Answer is checked to adhere  to ISOTP, but not check the response for an  UDS header. Max command length is 7 bytes.
- /Raw?f=x,y Send hex command y to ID x. Max command length is 8 bytes.
- /Config This is defuct and should not be used

All answers are JSON strings with the elements, "C", "F" and "R". C represents the comand, F the frame and R the result. The result is one of
- a hex string (for FreeFrame and IsoTpFrame
- the literal OK
- a human readable error that always starts with a dash

This interface is compatible with the CanZE app as an alternative to the ELM327 driver. There is also a WiFi to Bluetooth (ELM327) sketch available using the same interface, as well as as a simple PHP script emulating the car.
