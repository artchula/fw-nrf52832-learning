import processing.serial.*;


PrintWriter output;


Serial myPort;        // The serial port
int xPos = 1;         // horizontal position of the graph
float inByte = 0;

float h_buf[] = new float[1200];
int new_inbyte_ax[] = new int[1200];
int new_inbyte_ay[] = new int[1200];
int new_inbyte_az[] = new int[1200];
int count = 0;
int flag_display = 0;
int max_x;
int min_x;

int max_y;
int min_y;

int max_z;
int min_z;
 
int acc_x;
int acc_y;
int acc_z;

  void setup () {
    
     output = createWriter("acc_fall_4.txt");
     
    // set the window size:
    size(1200, 600);

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
    
    flag_display = 0;
    min_x = 100000;//–32768;8
    max_x = -100000;
    
    min_y = 100000;//–32768;8
    max_y = -100000;
    
    min_z = 100000;//–32768;8
    max_z = -100000;
    //noSmooth();
  }

  void draw () {
    if(flag_display == 1)
    {
        // draw the line:
       
        
        background(0);
        
        for(int i = 0;i<1199;i++)
        {
            //line(i, new_inbyte[i], i, new_inbyte[i]+1);   
            stroke(255, 0, 0);
            //line(i, new_inbyte_ax[i], i+1, new_inbyte_ax[i]+1);   
            point(i, new_inbyte_ax[i]);
            stroke(0, 255, 0);
            //line(i, new_inbyte_ay[i], i+1, new_inbyte_ay[i]+1);   
            point(i, new_inbyte_ay[i]);
            stroke(0, 0, 255);
            //line(i, new_inbyte_az[i], i+1, new_inbyte_az[i]+1);   
            point(i, new_inbyte_az[i]);
        }
        flag_display = 0;
    }
  }

  void serialEvent (Serial myPort) {
    // get the ASCII string:
    String inString = myPort.readStringUntil('\n'); 

    String[] list = split(inString, ',');
    
    if(list.length == 4)
    {
      output.print(inString);
      try{
           acc_x = int(list[0]);
           acc_y = int(list[1]);
           acc_z = int(list[2]);
          
                  
              if(acc_x < min_x)
           {
               min_x =acc_x; 
           }
           if(acc_x > max_x)
           {
               max_x = acc_x;
           }
           
           if(acc_y < min_y)
           {
               min_y =acc_y; 
           }
           if(acc_y > max_y)
           {
               max_y = acc_y;
           }
           
           if(acc_z < min_z)
           {
               min_z =acc_z; 
           }
           if(acc_z > max_z)
           {
               max_z = acc_z;
           }
           
           
           for(int i=1199;i>0;i--)
           {
              new_inbyte_ax[i] =new_inbyte_ax[i-1];
              new_inbyte_ay[i] =new_inbyte_ay[i-1];
              new_inbyte_az[i] =new_inbyte_az[i-1];
      
     
           }
           
           
           new_inbyte_ax[0] = int(map(acc_x, min_x, max_x, 0, height));
           new_inbyte_ay[0] = int(map(acc_y, min_y, max_y, 0, height));
           new_inbyte_az[0] = int(map(acc_z, min_z, max_z, 0, height));
           flag_display = 1;
           
           println(acc_x+","+acc_y+","+ acc_z + "h:" + new_inbyte_ax[0]);
           
      }catch (Exception e)
      {
          return;
      }
      
      
    }
    
   
}

void keyPressed() {
  output.flush(); // Writes the remaining data to the file
  output.close(); // Finishes the file
  exit(); // Stops the program
  println("Close");
}