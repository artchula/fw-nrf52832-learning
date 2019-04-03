import processing.serial.*;

  Serial myPort;        // The serial port
  int xPos = 1;         // horizontal position of the graph
  float inByte = 0;

  float h_buf[] = new float[500];
  float new_inbyte[] = new float[500];
  int count = 0;
  int flag_display = 0;
 float max;
 float min;

  void setup () {
    // set the window size:
    size(80, 300);

    // List all the available serial ports
    // if using Processing 2.1 or later, use Serial.printArray()
    println(Serial.list());

    // I know that the first port in the serial list on my Mac is always my
    // Arduino, so I open Serial.list()[0].
    // Open whatever port is the one you're using.
    myPort = new Serial(this, Serial.list()[0], 115200);

    // don't generate a serialEvent() unless you get a newline character:
    myPort.bufferUntil('\n');

    // set initial background:
    background(0);
  }

  void draw () {
    if(flag_display == 1)
    {
        // draw the line:
        stroke(127, 34, 255);
        
        background(0);
        
        for(int i = 0;i<80;i++)
        {
            line(i, new_inbyte[i], i, new_inbyte[i]+1);    
        }
        //
        
        //// at the edge of the screen, go back to the beginning:
        //if (xPos >= width) {
        //  xPos = 0;
        //  background(0);
        //} else {
        //  // increment the horizontal position:
        //  xPos++;
        //}
        flag_display = 0;
        
        
         
    }
    
  }

  void serialEvent (Serial myPort) {
    // get the ASCII string:
    String inString = myPort.readStringUntil('\n');

    if (inString != null) {
      for(int i=399;i>0;i--)
      {
          h_buf[i] =h_buf[i-1];
      }
      // trim off any whitespace:
      inString = trim(inString);
      // convert to an int and map to the screen height:
     
      inByte = float(inString);
      
      if(inByte>0.0&&inByte<900000)
      {
        
          float sum=0;
          sum = sum + inByte;
          for(int j=1;j<3;j++)
          {
              sum = sum + h_buf[j];
          }
          
          h_buf[0] = sum/3;
          
          max = 0;
          min = 999999;
          for(int i=0;i<80;i++)
          {
              if(h_buf[i] > max)
              {
                  max = h_buf[i];
              }
              
              if(h_buf[i] < min)
              {
                  min = h_buf[i]; 
              }
          }
          for(int i=0;i<80;i++)
          {
              new_inbyte[i] = map(h_buf[i], min, max, 0, height);
          }
          //println(inByte+","+min+","+max+" >>"+new_inbyte[0]);
          flag_display = 1;
          getHeartRate();
      }
    }
}

int state_min    = 0;
int state_count  = 1;
int state_max    = 2;

int flag_state   = 0;
int buffer_index = 79;
long delta_time  = 0;
float heartRate  = 0;
float avr_heartRate  = 0;
float mov_heartRate[] = new float[50];
int count_mov = 0;
int flag_avr = 0;

void getHeartRate()
{
    //if(delta_time > 60)
    //{
    //    flag_state = state_min;
    //}
    
    switch(flag_state)
    {
        case 0:
           if(findMinPoint(buffer_index) == 1)
           {
               flag_state = 1;
               delta_time = 0;
               println("Found MIN");
           }else{
               flag_state = 0;
           }
           break;
        case 1:
           if(findMaxPoint(buffer_index) == 1)
           {
               flag_state = 0;
               heartRate = 60000/(25*delta_time*2);
               if(delta_time<5){
                   break;
               }
               mov_heartRate[count_mov] = heartRate;
               
               count_mov++;
               
               if(count_mov>=50)
               {
                   count_mov = 0;
                   flag_avr = 1;
               }
               
               
               if(flag_avr == 1)
               {
                   for(int i=0;i<50;i++)
                   {
                       avr_heartRate +=  mov_heartRate[i];
                   }
                   avr_heartRate = avr_heartRate/50;
               }else{
                   for(int i=0;i<count_mov;i++)
                   {
                       avr_heartRate +=  mov_heartRate[i];
                   }
                   avr_heartRate = avr_heartRate/count_mov;
               }
               println("Found HR >> " + delta_time +"heartRate "+ heartRate + " HR Avr::" + avr_heartRate );
           }else{
               flag_state = 1;
               delta_time++;
           }
           break;
    }
}

int findMinPoint(int lastIndex)
{
    if( new_inbyte[lastIndex-4] < new_inbyte[lastIndex-3]  && new_inbyte[lastIndex-3] < new_inbyte[lastIndex-2]  &&  new_inbyte[lastIndex-2] > new_inbyte[lastIndex-1] &&  new_inbyte[lastIndex-1] > new_inbyte[lastIndex])
    //if( new_inbyte[lastIndex-3] < new_inbyte[lastIndex-2] && new_inbyte[lastIndex-2] < new_inbyte[lastIndex-1]  && new_inbyte[lastIndex-1] >  new_inbyte[lastIndex])
    //if( new_inbyte[lastIndex-2] < new_inbyte[lastIndex-1]  && new_inbyte[lastIndex-1] >  new_inbyte[lastIndex])
    {
        return 1;
    }
    return 0;
}

int findMaxPoint(int lastIndex)
{
    if(new_inbyte[lastIndex-2] > new_inbyte[lastIndex-1] && new_inbyte[lastIndex-1] < new_inbyte[lastIndex])
    {
        return 1;
    }
    return 0;
}