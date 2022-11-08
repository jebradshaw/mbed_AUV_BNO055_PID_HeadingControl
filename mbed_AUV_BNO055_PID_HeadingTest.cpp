#include "mbed.h"
#include "BNO055.h"
#include "PID.h"

#define PI 3.1415926545359f 

//+/- 3*PI radian, +/-540 degrees for controller wraparound swing at zero crossing (360 deg is not enough)
#define YAW_MIN_SP      (-3.0*PI)          // Min Yaw in radians
#define YAW_MAX_SP      (3.0*PI)         // Max Yaw in radians
#define YAW640_RATIO    (.981746f)      //for some reason, the YAW RADIAN measurement from the BNO
                                        // yeilds 0.0 - 6.40 instead of 2*PI
//Instantiate classes
Serial pc(USBTX, USBRX);
BNO055 bno(p28, p27);   // (SDA, SCL) or... SDA is p28, SCL is p27

PID pidYaw(0.0, 0.0, 0.0, .02);

Ticker tickerPulse;

PwmOut led_pulse(LED1);

//yaw/heading controller global variables
Ticker tickerYaw;
volatile float yaw_Tdelay = .02;
volatile float yaw_Pk =10.5f;
volatile float yaw_Ik =0.0f;      //.055
volatile float yaw_Dk =0.0;
volatile float yaw_sp = 0.0;      //initialize at 0.0 rad position pitch
volatile float yaw_co = 0.0;
volatile float yaw_sign=0.0;
volatile float yaw_error=0.0;     //make global for now for testing purposes
volatile float yaw_err, yaw_P, yaw_I, yaw_D;

volatile float yaw_err_last=0.0;     //make global for now for testing purposes
volatile float yaw_temp=0.0;     //make global for now for testing purposes
volatile float yaw_cor=0.0;      //corrected yaw heading in radians

volatile float yaw_temp_error=0.0;
volatile float yaw_temp_heading = 0.0;


char calib_local[] = {0x00,0x00,
                0x00,0x00,
                0x00,0x00,
                0x28,0x00,
                0x42,0x00,
                0x15,0x01,
                0x00,0x00,
                0x00,0x00,
                0x00,0x00,
                0xE8,0x03,
                0x84,0x02};                        

void yawController(void);
void heartbeat(void);
void bno_init(void);
void display_commands(void);
void init_yawController(void);

int con_state;
float yaw_err_unwrapped;
//-------- FUNCTIONS ----------------------                        
void yawController(void){
 //first check for the sign, which direction is faster to turn in
    
    //set the set point
    pidYaw.setSetPoint(yaw_sp);
    
    yaw_err_unwrapped = yaw_sp - yaw_cor;
    
    if(yaw_sp >= yaw_cor){                   //If the set point is greater then the corrected heading
        yaw_error = yaw_sp - yaw_cor;       //get the difference
        
        if(yaw_error <= PI){        //Turn left
            con_state = 1;
            yaw_sign=1.0; 
            
            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2*PI;
              
             //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp + yaw_error;
            else
                yaw_temp_error = yaw_sp - yaw_error;       
        }                  
        else if(yaw_error > PI){       //Turn right
            con_state = 2;
            yaw_sign=-1.0;
            
            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2*PI;
              
             //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp - yaw_error;
            else
                yaw_temp_error = yaw_sp + yaw_error; 
        }
    
        //Update the process variable.
        pidYaw.setProcessValue(yaw_temp_error);
    }    
    else if(yaw_sp < yaw_cor){
        yaw_error = yaw_cor - yaw_sp;
        if(yaw_error <= PI){    //difference is
            con_state = 3; 
            yaw_sign=-1.0;              //Turn left

            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2*PI;
                            //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp - yaw_error;
            else
                yaw_temp_error = yaw_sp + yaw_error;                    
        }
        else if(yaw_error > PI){   //180
            con_state = 4;
            yaw_sign=1.0;           //turn right
            //is the error < -PI
            if((yaw_error < 0.0) && (fabs(yaw_error) > PI))
              yaw_error += 2*PI;
                            //is the error < -PI
            if((yaw_error > 0.0) && (fabs(yaw_error) > PI))
              yaw_error = 2*PI - yaw_error;
            
            //calculate the heading offset from error relative to setpoint for controller  
            if(yaw_error < 0.0)
                yaw_temp_error = yaw_sp + yaw_error;
            else
                yaw_temp_error = yaw_sp - yaw_error;
        }
        //Update the process variable.
        pidYaw.setProcessValue(yaw_temp_error);
    }
        
    //Set the new output.
    yaw_co = pidYaw.compute();        
}

void heartbeat(void){
    static int flag;        
    static int cntr;
    
    switch(flag){        
        case 0:        
            led_pulse = led_pulse + .2;
            if(led_pulse >= 1.0)
                flag = 1;
            break;
        case 1:    
            led_pulse = led_pulse - .2;
            if(led_pulse <= 0.0)
                flag = 2;            
            break;
        case 2:        
            led_pulse = led_pulse + .2;
            if(led_pulse >=1.0)
                flag = 3;
            break;
        case 3:    
            led_pulse = led_pulse - .2;
            if(led_pulse <= .0)
                flag = 4;
            break;
        case 4:
            cntr++;
            if(cntr>40){
                cntr=0;
                flag = 0;    
            }
            break;
        default:
            cntr=0;
            flag=0;
            break;
    }//switch   
}

void bno_init(void){    
    if(bno.check()){
        pc.printf("BNO055 connected\r\n");
        bno.reset();
        wait(.5);
        bno.setmode(OPERATION_MODE_CONFIG);
        bno.SetExternalCrystal(1);
        //bno.set_orientation(1);
        bno.setmode(OPERATION_MODE_NDOF);  //Uses magnetometer
        //bno.setmode(OPERATION_MODE_NDOF_FMC_OFF);   //no magnetometer
        bno.set_angle_units(RADIANS);
        
        //write calibration data
        //for(int i=0;i<22;i++){
        //    bno.calibration[i] = calib_local[i];
        //}
        //bno.write_calibration_data();
    }
    else{
        tickerPulse.attach(&heartbeat, .02);    //run pitch controller every 20 milliseconds
        while(1){
            pc.printf("BNO055 NOT connected\r\n Program Trap.\r\n");
            wait(1);
        }
    }    
}                        

//------ ASCII command set ----------------------------
void display_commands(void){
    pc.printf("\r\n Command set\r\n");
    pc.printf("? or help - This menu\r\n");    
    wait(.01);
    pc.printf("bnomode - changes orientation mode of IMU\r\n");    
    pc.printf("bnoreadcal - read he calibration data from bno IMU\r\n");
    pc.printf("bnowritecal - write the calibration data\r\n");
    wait(.01);
    
    //wait to receive character before exiting
    while(!pc.readable()){                                 
        char c = pc.getc();
            return;
    }
}

void init_yawController(void){
    //resets the controllers internals
    pidYaw.reset();
    //input limits for pitch controller
    pidYaw.setInputLimits(YAW_MIN_SP, YAW_MAX_SP);   //+/- 2*PI radian, +/-360 degrees for controller wraparound swing
    //Servo Output -1.0 to 1.0
    pidYaw.setOutputLimits(-0.85, 0.85); //+/- .72
    //If there's a bias.
    pidYaw.setBias(0.0);
    pidYaw.setMode(AUTO_MODE);
    
    pidYaw.setInterval(yaw_Tdelay);
      
    //We want the process variable to be 0.0 at start
    pidYaw.setSetPoint(yaw_sp);
    pidYaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
    
    
    tickerYaw.attach(&yawController, .02);    //run pitch controller every 20 milliseconds
}
// ---------------- MAIN ------------------------------ 
int main() {
    char cmd_str[30];
    
    wait(.5);       //allow time for voltages to settle after micro awakens
    pc.baud(115200);
    
    pc.printf("%s\r\n", __FILE__);  //serial transmit of file name
    wait(.5);
    bno_init();     //Initialize the BNO055 IMU

    tickerPulse.attach(&heartbeat, .02);    //run pitch controller every 20 milliseconds
    init_yawController();
    
    while(1) {
        bno.get_angles();
        yaw_cor = bno.euler.yaw * YAW640_RATIO; //correct the yaw for radian measurement
        if(yaw_cor > PI)
            yaw_cor = -(PI - yaw_cor) - PI;
        bno.get_calib();
        //bno.get_accel();
        
        //pc.printf("T:%02d:%02d:%02d:%02d:%03d ", runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms);
        //pc.printf("%02Xh %.2f %.2f %.2f ", bno.calib, bno.euler.roll, bno.euler.pitch, yaw_cor);
        //pc.printf("pitch_co=%.3f yaw_co=%.3f ", pitch_co, yaw_co);
        //pc.printf("Roll=%6.3f Pitch=%6.3f  Yaw=%6.3f ", -bno.euler.roll, bno.euler.pitch, bno.euler.yaw);
        //pc.printf("STAT= S%1d G%1d A%1d M%1d ", sys_stat,gyr_stat,acc_stat,mag_stat);
        //pc.printf("Accel X = %.3f Accel Y = %.3f Accel Z = %.3f ", bno.accel.x, bno.accel.y, bno.accel.z);        
        pc.printf("sp=%.3f cor=%.3f yawT_error=%.3f, co=%.3f yaw_error=%.3f yaw_sign=%.1f %d errU=%.3f", 
                    yaw_sp, yaw_cor, yaw_temp_error, yaw_co, yaw_error, yaw_sign, con_state, yaw_err_unwrapped);
        //pc.printf("yaw_err=%.3f yaw_sign=%.3f ", yaw_temp, yaw_sign);
        pc.printf("\r\n");
        
        wait(.05);
        
        if(pc.readable()){            //if data in receive buffer        
            pc.scanf("%s", cmd_str);//read the string
            
            //display the command set
            if(!strcmp(cmd_str, "?") || !strcmp(cmd_str, "help")){
                display_commands();
            }
            //change bno orientation mode
            if(!strcmp(cmd_str, "bnomode")){
                int mode=0;
                pc.printf("\r\nbno orientation\r\nEnterMode:");
                pc.scanf("%d", &mode);//read the string
                pc.printf("\r\n%d orientation mode entered", mode);
                
                wait(1);
                bno.reset();
                wait(.5);
                bno.setmode(OPERATION_MODE_CONFIG);
                bno.SetExternalCrystal(1);
                bno.set_orientation(mode);
                bno.setmode(OPERATION_MODE_NDOF);  //Uses magnetometer
                //bno.setmode(OPERATION_MODE_NDOF_FMC_OFF);   //no magnetometer
                bno.set_angle_units(RADIANS);
                wait(.02);           
            }
            // read the bno calibration values
            if(!strcmp(cmd_str, "bnoreadcal")){
                bno.read_calibration_data();
                pc.printf("Calibration data read is as follows...\r\n");
                for(int i=0;i<22;i++){
                    pc.printf("reg=0x%2X byte[%d]=0x%02X\r\n", i+55, i, bno.calibration[i]);
                    wait(.02);
                }
                
                pc.printf("\r\n\r\nCopy the C-Array below.\r\n");
                pc.printf("\r\nchar calib_local[] = {");
                for(int i=0;i<22;i+=2){
                    pc.printf("0x%02X,0x%02X", bno.calibration[i], bno.calibration[i+1]);                                        
                    if(i==20)
                        pc.printf("};\r\n");
                    else
                        pc.printf(",\r\n\t\t\t");
                    wait(.02);
                }
                
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }
            
     
            //---------------- YAW ---------------------------------
            //Turn ON the Yaw controller
            if(!strcmp(cmd_str, "yawon")){
                
                init_yawController();
                pc.printf("Yaw Controller is ON\r\n");
                wait(1.0);
            }

            //Turn OFF the Yaw controller
            if(!strcmp(cmd_str, "yawoff")){
                
                tickerYaw.detach();    //run pitch controller every 20 milliseconds
                pc.printf("Yaw Controller is OFF\r\n");
                wait(1.0);
            }
                        
            //change the yaw proportional gain
            if(!strcmp(cmd_str, "yawp")){
                float Pk_temp;
                pc.printf("Set yaw proportional gain (currently %.3f)\r\n", yaw_Pk);
                pc.scanf("%f", &Pk_temp);
                yaw_Pk = Pk_temp;
                
                pidYaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
                
                pc.printf("Yaw Proportional gain is %.3f\r\n", yaw_Pk);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }
            
            //change the integral gain for the yaw controller
            if(!strcmp(cmd_str, "yawi")){
                float Ik_temp;
                pc.printf("Set yaw integral gain (currently %.3f)\r\n", yaw_Ik);
                pc.scanf("%f", &Ik_temp);
                yaw_Ik = Ik_temp;
                
                pidYaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
                
                pc.printf("yaw integral gain is %.3f\r\n", yaw_Ik);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }            
            
            //change the derivative gain for the yaw controller
            if(!strcmp(cmd_str, "yawd")){
                float Dk_temp;
                pc.printf("Set yaw derivative gain (currently %.3f)\r\n", yaw_Dk);
                pc.scanf("%f", &Dk_temp);
                yaw_Dk = Dk_temp;
                
                pidYaw.setTunings(yaw_Pk, yaw_Ik, yaw_Dk);
                
                pc.printf("yaw derivative gain is %.3f\r\n", yaw_Dk);
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }            
            
            //change the set point on the yaw controller
            if(!strcmp(cmd_str, "yawsp")){
                pc.printf("Set yaw (currently %.3f)\r\n", yaw_sp);
                pc.scanf("%f", &yaw_sp);
                
                if(yaw_sp < YAW_MIN_SP){
                    yaw_sp = YAW_MIN_SP;
                    pc.printf("yaw Entered Exceeds min value of %.3f\r\n", YAW_MIN_SP);
                }
                if(yaw_sp > YAW_MAX_SP){
                    yaw_sp = YAW_MAX_SP;
                    pc.printf("yaw Entered Exceeds max value of %.3f\r\n", YAW_MAX_SP);
                }
                
                pc.printf("Yaw is %.3f\r\n", yaw_sp);
                
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;
            }            
            
            //write calibration data
            if(!strcmp(cmd_str, "bnowritecal")){
                for(int i=0;i<22;i++){
                    bno.calibration[i] = calib_local[i];
                }
                bno.write_calibration_data();
                pc.printf("Calibration data written is as follows...\r\n");
                for(int i=0;i<22;i++){
                    pc.printf("reg=0x%2X byte[%d]=0x%02X\r\n", i+55, i, bno.calibration[i]);
                    wait(.02);
                }
                pc.printf("\r\n\r\nPress key to continue.\r\n");
                while(!pc.readable());
                char c = pc.getc();
                c=0;                
            }
        }//if pc.readable()
        
    }//while(1)
}//main()
