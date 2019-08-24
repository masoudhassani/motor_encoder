void commandInterpreter(String strCommand)
{
    uint8_t cmdLength = strCommand.length();
    String sub2;
    int8_t ind;
    uint8_t intLength = 1;
    // interpret commands starting with 'g' or 'G'
    if(strCommand.charAt(0)=='G' || strCommand.charAt(0)=='g'){

        // read the integer after the command definition. intLength digit(s) int is expected
        uint8_t sub1 = strCommand.substring(1,intLength+1).toInt();

        switch(sub1)
        {
            case 0:
                Serial.print("Current angle is: ");Serial.print(currentAngle);
                Serial.print('\n');
                break;

            case 1:
                Serial.print("Current PID gains are: ");Serial.print(pGain,4);
                Serial.print('\t');Serial.print(iGain,4);Serial.print('\t');
                Serial.print(dGain,4);Serial.print('\n');
                break;

            // stuff about position, speed and acceleration
            case 2:
                sub2 =  strCommand.substring(intLength+1,cmdLength);
                ind = sub2.length();
                // set setpoint angle
                if (sub2.charAt(0)=='A' || sub2.charAt(0)=='a'){
                    setpointAngle = sub2.substring(1,ind).toFloat();
                }
                // limit forward speed, 1 is max
                else if (sub2.charAt(0)=='F' || sub2.charAt(0)=='f'){
                    maxEffort = sub2.substring(1,ind).toFloat();
                    // need a setter function in the pid library
                }
                // limit forward speed, -1 is min
                else if (sub2.charAt(0)=='R' || sub2.charAt(0)=='r'){
                    minEffort = sub2.substring(1,ind).toFloat();
                    // need a setter function in the pid library
                }
                break;

            // stuff about pid controller
            case 3:
                sub2 =  strCommand.substring(intLength+1,cmdLength);
                ind = sub2.length();
                // set p gain
                if (sub2.charAt(0)=='P' || sub2.charAt(0)=='p'){
                    pGain = sub2.substring(1,ind).toFloat();
                }
                // set i gain
                else if (sub2.charAt(0)=='I' || sub2.charAt(0)=='i'){
                    iGain = sub2.substring(1,ind).toFloat();
                }
                // set d gain
                else if (sub2.charAt(0)=='D' || sub2.charAt(0)=='d'){
                    dGain = sub2.substring(1,ind).toFloat();
                }
                break;

            // stuff about the encoder
            case 4:
                sub2 =  strCommand.substring(intLength+1,cmdLength);
                ind = sub2.length();
                // set gear ratio
                if (sub2.charAt(0)=='M' || sub2.charAt(0)=='N'){
                    gearRatio = sub2.substring(1,ind).toFloat();
                }
                // set ppr
                else if (sub2.charAt(0)=='N' || sub2.charAt(0)=='n'){
                    ppr = sub2.substring(1,ind).toFloat();
                }
                break;

            case 5:
                setpointAngle = -90;
                break;

            case 6:
                setpointAngle = 90;
                break;
        }
    }
}
