##ser = serial.Serial('/dev/rfcomm0',9600,timeout=1)
##while 1:
##    ch0 = ser.readline()
##    sw = ch0.decode("utf-8")
##    sw = str(sw).strip(' \t\n\r')
##    print type(sw),sw
##    if sw ==  '0':
##        print 'off'
##    elif sw =='1':
##        print 'on'

def bt(ser,sw):
    ser.reset_input_buffer()
    ch0 = ser.readline()
    temp_sw = ch0.decode("utf-8")
    temp_sw = str(temp_sw).strip(' \t\n\r')
    if temp_sw == '1' or temp_sw == '0':
        sw = temp_sw
    print type(sw),sw
    return(sw)
