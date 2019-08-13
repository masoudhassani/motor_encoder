void commandInterpreter(String strCommand)
{
    uint8_t cmdLength = strCommand.length();
    // interpret commands starting with 'g' or 'G'
    if(strCommand.charAt(0)=='G' || strCommand.charAt(0)=='g'){

        // read the integer after the command definition. two digit int is expected
        uint8_t sub1 = strCommand.substring(1,3).toInt();

        switch(sub1)
        {
            case 0:
                Serial.print("Current PID gains are: ");Serial.print(pGain,4);
                Serial.print('\t');Serial.print(iGain,4);Serial.print('\t');
                Serial.print(dGain,4);Serial.print('\n');
                break;

            case 1:
                setpointAngle = -90;
                break;

            case 2:
                setpointAngle = 90;
                break;

            case 3:
                String sub2 =  strCommand.substring(3,cmdLength);
                int8_t ind = sub2.length();
                if (sub2.charAt(0)=='P' || sub2.charAt(0)=='p'){
                    pGain = sub2.substring(1,ind).toFloat();
                }
                else if (sub2.charAt(0)=='I' || sub2.charAt(0)=='i'){
                    iGain = sub2.substring(1,ind).toFloat();
                }
                else if (sub2.charAt(0)=='D' || sub2.charAt(0)=='d'){
                    dGain = sub2.substring(1,ind).toFloat();
                }
                break;
        }
    }
}
