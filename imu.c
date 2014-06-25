
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>

#include "mpu9150.h"
#include "linux_glue.h"
#include "local_defaults.h"
/**************************************************/

#define CR         0x0d
#define LF         0x0a
#define ESC        0x1b
#define BEEP       0x07

#define SPACE	  0x20
#define COMMA	  0x2C
#define MAXSIZE    100		/* GPS at most, sends 80 or so chars per message string.  So set maximum to 100 */

#include <stdio.h>
#include <ctype.h>  		/* required for the isalnum function */
#include <stdlib.h>
#include <string.h>
//#include <conio.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/ioctl.h>
#include <wiringPi.h>
#include <wiringSerial.h>
/****************************************************/
int set_cal(int mag, char *cal_file);
void read_loop(unsigned int sample_rate);
void print_fused_euler_angles(mpudata_t *mpu);
void print_fused_quaternion(mpudata_t *mpu);
void print_calibrated_accel(mpudata_t *mpu);
void print_calibrated_mag(mpudata_t *mpu);
void register_sig_handler();
void sigint_handler(int sig);
int gps();
void rotate_az(int step_az, float speed_az);
void rotate_al(int step_al, float speed_al);
float lat,longi,utc,l,d,gmst0,ra,dec,ha,lst,alt,az,d,lati,ra_e,dec_e,ra_obj,dec_obj,ha_obj,al_obj,az_obj,al_e,az_e,step_az,step_al;
int done,aa,it;
int i;
int pin_trigger=3;
int pin_missionend=4;
//22 AND 23
int pin_dir_az=5;
int pin_cntrl_az=6;
int pin_dir_al=1;
int pin_cntrl_al=2;
float gps_v[4];
float e_x,e_y,e_z,ra_h,alti;
FILE *radecFile;
int fd;
char obj[];
	

void usage(char *argv_0)
{
	printf("\nUsage: %s [options]\n", argv_0);
	printf("  -b <i2c-bus>          The I2C bus number where the IMU is. The default is 1 to use /dev/i2c-1.\n");
	printf("  -s <sample-rate>      The IMU sample rate in Hz. Range 2-50, default 10.\n");
	printf("  -y <yaw-mix-factor>   Effect of mag yaw on fused yaw data.\n");
	printf("                           0 = gyro only\n");
	printf("                           1 = mag only\n");
	printf("                           > 1 scaled mag adjustment of gyro data\n");
	printf("                           The default is 4.\n");
	printf("  -a <accelcal file>    Path to accelerometer calibration file. Default is ./accelcal.txt\n");
	printf("  -m <magcal file>      Path to mag calibration file. Default is ./magcal.txt\n");
	printf("  -v                    Verbose messages\n");
	printf("  -h                    Show this help\n");

	printf("\nExample: %s -b3 -s20 -y10\n\n", argv_0);
	
	exit(1);
}

int main(int argc, char **argv)
{
	radecFile=fopen("radecData.txt","a");
	fd=serialOpen("/dev/ttyAMA0",9600);
	int opt, len;
	//printf("Énter the object: \n");
	//scanf("%s", obj);
	//fprintf(radecFile," %s \n",obj);

        if (wiringPiSetup() == -1)
 	{
         printf("exit");
	 exit(1);
	}
        pinMode(pin_trigger, INPUT);
        pinMode( pin_missionend, OUTPUT);
        if(digitalRead(pin_trigger) == 1)
        {
         delay(100);
	digitalWrite( pin_missionend,HIGH );	   
       }
	int i2c_bus = DEFAULT_I2C_BUS;
	int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
	int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
	int verbose = 0;
	char *mag_cal_file = NULL;
	char *accel_cal_file = NULL;

	while ((opt = getopt(argc, argv, "b:s:y:a:m:vh")) != -1) {
		switch (opt) {
		case 'b':
			i2c_bus = strtoul(optarg, NULL, 0);
			
			if (errno == EINVAL)
				usage(argv[0]);
			
			if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS)
				usage(argv[0]);

			break;
		
		case 's':
			sample_rate = strtoul(optarg, NULL, 0);
			
			if (errno == EINVAL)
				usage(argv[0]);
			
			if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE)
				usage(argv[0]);

			break;

		case 'y':
			yaw_mix_factor = strtoul(optarg, NULL, 0);
			
			if (errno == EINVAL)
				usage(argv[0]);
			
			if (yaw_mix_factor < 0 || yaw_mix_factor > 100)
				usage(argv[0]);

			break;

		case 'a':
			len = 1 + strlen(optarg);

			accel_cal_file = (char *)malloc(len);

			if (!accel_cal_file) {
				perror("malloc");
				exit(1);
			}

			strcpy(accel_cal_file, optarg);
			break;

		case 'm':
			len = 1 + strlen(optarg);

			mag_cal_file = (char *)malloc(len);

			if (!mag_cal_file) {
				perror("malloc");
				exit(1);
			}

			strcpy(mag_cal_file, optarg);
			break;

		case 'v':
			verbose = 1;
			break;

		case 'h':
		default:
			usage(argv[0]);
			break;
		}
	}

	register_sig_handler();

	mpu9150_set_debug(verbose);

	if (mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor))
		exit(1);

	set_cal(0, accel_cal_file);
	set_cal(1, mag_cal_file);

	if (accel_cal_file)
		free(accel_cal_file);

	if (mag_cal_file)
		free(mag_cal_file);

	read_loop(sample_rate);

	mpu9150_exit();

	return 0;
	fclose(radecFile);
}

void read_loop(unsigned int sample_rate)
{
	
	unsigned long loop_delay;
	mpudata_t mpu;

	memset(&mpu, 0, sizeof(mpudata_t));

	if (sample_rate == 0)
		return;

	loop_delay = (1000 / sample_rate) - 2;

	printf("\nEntering read loop (ctrl-c to exit)\n\n");

	linux_delay_ms(loop_delay);
	 

	while (!done) {
		if (mpu9150_read(&mpu) == 0) {
			print_fused_euler_angles(&mpu);
			// printf_fused_quaternions(&mpu);
			// print_calibrated_accel(&mpu);
			// print_calibrated_mag(&mpu);
                        
		}
               	linux_delay_ms(loop_delay);
	}
	
	printf("\n\n");
}
void print_fused_euler_angles(mpudata_t *mpu)
{
		
    
//////////////////////////////////////////////////////////////////  
			printf("\nThe MPU:\n\rX: %0.1f Y: %0.2f Z: %0.0f        ",
			mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE);
			e_x=mpu->fusedEuler[VEC3_X]* RAD_TO_DEGREE;
			e_y=mpu->fusedEuler[VEC3_Y]* RAD_TO_DEGREE;
			e_z=mpu->fusedEuler[VEC3_Z]* RAD_TO_DEGREE;
			//printf("ours \nX: %0.2f Y: %0.0f Z: %0.0f  \n ",e_x,e_y,e_z);
			az=e_z;
			alt=e_x;
			if(az < 0)
			  {
			   az=360.0+az;
			  }
			az=az-90;
			if(az < 0)
			  {
			   az=360.0+az;
			  }
			if(alt > 90) 
			  {
			   alt=180-alt;
			  }
			alt=alt-2.01;
			printf("AZ: %f   AL:%f \n",az,alt);
			az=az*3.14159265/180;
			alt=alt*3.14159265/180;
			gps();
                        delay(100);
                         /*
	                
			if((gps_v[0] < 90 && gps_v[0] > 0) &&  (gps_v[1] > 0 && gps_v[1] < 180))
			  {
			   lat=gps_v[0];
			   longi=gps_v[1];
			   //utc=gps_v[2];
			   alti=gps_v[3];
			   }
			if(gps_v[2] >= 0 && gps_v[2] <= 24)
			  {
				printf("ínside gps\n");
			   utc=gps_v[2];
			  }
                              */
		         
			//lat=12.9833;
			//longi=77.5833;
			//utc=7.2;
			d=367*2013-7*(2013+(10+9)/12)/4+275*10/9 +13-730530;
			d=d+utc/24.0;
			l=282.9404+(4.70935E-5)*d+356.0470+0.9856002585*d;
			gmst0=l+180;
			lst=gmst0+utc*15.0+longi;
			d=lst/360;
			it=(int)d;
			lst=lst-it*360;
			lati=lat;
			lat=lat*3.14159265/180;
			dec=asin((sin(lat)*sin(alt))+(cos(lat)*cos(alt)*cos(az)));
                        ha=acos(sin(alt)/(cos(dec)*cos(lat))-tan(dec)*tan(lat));
  			//ha=acos(sin(alt)-(sin(dec)*sin(lat)))/(cos(dec)*cos(lat));
                        ha=ha* RAD_TO_DEGREE;
                        //ha=360-ha;
 			dec=dec* RAD_TO_DEGREE;
  			az=az*RAD_TO_DEGREE;;
			alt=alt*RAD_TO_DEGREE;
			ra=lst-ha;
			if(ra < 0)
			{
			 ra=360+ra;
			}
			printf("\nra:%f dec:%f ha:%f lst:%f\nlat:%f longi:%1f utc:%f \n",ra,dec,ha,lst,lati,longi,utc);
                        ra_h=ra/15.0;
			fprintf(radecFile, "ra:%f \t dec:%f \t lat:%f \t long:%f \t utc:%f \t alti:%f \t az:%f \t ele:%f\n", ra_h,dec,lati,longi,utc,alti,az,alt);
  			/*printf("Énter Ra and Dec\n");
			scanf("%f,%f",&ra_obj,&dec_obj);
			ha_obj=lst-ra_obj;
			al_obj=asin((sin(dec_obj)*sin(lat))+(cos(dec_obj)*cos(lat)*cos(ha_obj)));
			az_obj=acos((sin(dec_obj)/(cos(al_obj)*cos(lat)))-(tan(al_obj)*tan(lat)));
			al_obj=al_obj*RAD_TO_DEGREE;
			az_obj=az_obj*RAD_TO_DEGREE;
			az_e=az_obj-az;
			al_e=al_obj-alt;
			step_al=al_e*1600/360;
			step_az=az_e*1600/360;
			printf("in");
			if (wiringPiSetup() == -1)
 			   {
			    printf("exit");
			    exit(1);
			   }

			pinMode(pin_dir_al, OUTPUT);
			pinMode( pin_cntrl_al, OUTPUT);
			pinMode(pin_dir_az, OUTPUT);
			pinMode( pin_cntrl_az, OUTPUT);
			rotate_al(step_al, .01); 
  			rotate_az(step_az, .01);*/
			delay(1000);
			//delay(1000); 
                        //delay(1000); 
			//printf("rotated\n");
			fflush(stdout);

}

void print_fused_quaternions(mpudata_t *mpu)
{
	printf("\rW: %0.2f X: %0.2f Y: %0.2f Z: %0.2f",
			mpu->fusedQuat[QUAT_W],
			mpu->fusedQuat[QUAT_X],
			mpu->fusedQuat[QUAT_Y],
			mpu->fusedQuat[QUAT_Z]);

	fflush(stdout);
}

void print_calibrated_accel(mpudata_t *mpu)
{
	printf("\rX: %05d Y: %05d Z: %05d        ",
			mpu->calibratedAccel[VEC3_X], 
			mpu->calibratedAccel[VEC3_Y], 
			mpu->calibratedAccel[VEC3_Z]);

	fflush(stdout);
}

void print_calibrated_mag(mpudata_t *mpu)
{
	printf("\rX: %03d Y: %03d Z: %03d        ",
			mpu->calibratedMag[VEC3_X], 
			mpu->calibratedMag[VEC3_Y], 
			mpu->calibratedMag[VEC3_Z]);

	fflush(stdout);
}

int set_cal(int mag, char *cal_file)
{
	int i;
	FILE *f;
	char buff[32];
	long val[6];
	caldata_t cal;

	if (cal_file) {
		f = fopen(cal_file, "r");
		
		if (!f) {
			perror("open(<cal-file>)");
			return -1;
		}
	}
	else {
		if (mag) {
			f = fopen("./magcal.txt", "r");
		
			if (!f) {
				printf("Default magcal.txt not found\n");
				return 0;
			}
		}
		else {
			f = fopen("./accelcal.txt", "r");
		
			if (!f) {
				printf("Default accelcal.txt not found\n");
				return 0;
			}
		}		
	}

	memset(buff, 0, sizeof(buff));
	
	for (i = 0; i < 6; i++) {
		if (!fgets(buff, 20, f)) {
			printf("Not enough lines in calibration file\n");
			break;
		}

		val[i] = atoi(buff);

		if (val[i] == 0) {
			printf("Invalid cal value: %s\n", buff);
			break;
		}
	}

	fclose(f);

	if (i != 6) 
		return -1;

	cal.offset[0] = (short)((val[0] + val[1]) / 2);
	cal.offset[1] = (short)((val[2] + val[3]) / 2);
	cal.offset[2] = (short)((val[4] + val[5]) / 2);

	cal.range[0] = (short)(val[1] - cal.offset[0]);
	cal.range[1] = (short)(val[3] - cal.offset[1]);
	cal.range[2] = (short)(val[5] - cal.offset[2]);
	
	if (mag) 
		mpu9150_set_mag_cal(&cal);


	else 
		mpu9150_set_accel_cal(&cal);

	return 0;
}

void register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		exit(1);
	} 
}

void sigint_handler(int sig)
{
	done = 1;
}

int gps()
{
  unsigned char  charRead;	     		/* char read from COM port */
  unsigned char	 stringRead[MAXSIZE]; 		/* Buffer collects chars read from GPS */
  char  tempString[MAXSIZE];
  char  timeString[12];
  char  latitudeString[11];
  char  latitudeCardinalString[3];
  char  longitudeString[12];
  char  longitudeCardinalString[3]; 
  char  altitudeString[6];

  unsigned char  *pChar;
  unsigned char  dummyChar;

  unsigned  long utcTime;		/* Coordinated Universal Time */
  unsigned  long utcHour;
  unsigned  long utcMinutes;
  unsigned  long utcSeconds;
  unsigned  char lastCommaPosition;

  float     	 latitude;
  int	    	 latDegrees;
  float	    	 latMinutes;

  float 	 longitude;
  int		 longDegrees;
  float		 longMinutes;
  
  float          altitude;

  FILE           *gpsFile;	     		/* Text file of GPS strings read */
  unsigned int   j, k;				/* dummy variable */
  unsigned int	 i;		     		/* Number of chars read per GPS message string */
  unsigned int	 numLinesRead;        		/* Number of GPS strings read */

  dummyChar = 'A'; pChar = &dummyChar;
  gpsFile = fopen("gpsData.txt", "a");
  delay(100);  

//printf("\n Initializing port...... ");
  //comm_setting();
//printf(" Entering GPS done \n");

  numLinesRead = 0;



  //gordon code

//serial port wiringPi

//int fd;
//int data;
//fd=serialOpen("/dev/ttyAMA0",9600);
//data=serialGetchar(fd);
//printf("%dn",data);

// changes*****

/*
if(wiringPiSetup()== -1)
{
printf("unable to start wiringPi\n");
return 1;
}
*/


if((fd=serialOpen("/dev/ttyAMA0",9600))<0)
{
printf("error:unable to open serial device");
return 1;
}
else
{
printf("ttyAMAO \n");
}



// gps data reading starts from com port


  printf("Entering GPS...\n");
  do {
      charRead = (serialGetchar(fd));  	/* read char from serial port */
      if(charRead == '$') {     /* GPS messages start with $ char */
	  i = 0;
          //printf("ok");
	  numLinesRead++;
	  stringRead[i] = charRead;
	  do {

	     charRead = (serialGetchar(fd));
	     if( (charRead != '\0') && (isalnum(charRead) ||  isspace(charRead) || ispunct(charRead)) ) {
		i++;
		stringRead[i] = charRead;
	     }
	  } while(charRead != CR);
delay(10);
fprintf(gpsFile,"%s \n", stringRead);
	  /* By this point, a complete GPS string has been read so save it to file */
	  /* Append the null terminator to the string read */
	  stringRead[i+1] = '\0';

	  /* Analyze string that we collected */
	  j = 0;
	  pChar = stringRead;
	  while(*(pChar+j) != COMMA) {
	       tempString[j] = *(pChar+j);
	       j++;
	  }
	  tempString[j] = '\0';

	  /* Check if string we collected is the $GPGGA message */
	  if(tempString[3] == 'G' && tempString[4] == 'G' && tempString[5] == 'A') {
	      /*
		 Found GPGGA string.  It has 14 commas total.  Its NMEA sentence structure is:

		 $GPGAA,hhmmss.ss,ddmm.mmmm,n,dddmm.mmmm,e,q,ss,y.y,a.a,z,g.g,z,t.t,iii*CC
		 |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
		 0   	   1         2         3         4         5         6         7
		 0123456789012345678901234567890123456789012345678901234567890123456789012

		 where:

		 GPGAA		: GPS fixed data identifier
		 hhmmss.ss	: Coordinated Universal Time (UTC), also known as GMT
		 ddmm.mmmm,n	: Latitude in degrees, minutes and cardinal sign
		 dddmm.mmmm,e	: Longitude in degrees, minutes and cardinal sign
		 q		: Quality of fix.  1 = there is a fix
		 ss		: Number of satellites being used
		 y.y		: Horizontal dilution of precision
		 a.a,M		: GPS antenna altitude in meters
		 g.g,M		: geoidal separation in meters
		 t.t		: Age of the defferential correction data
		 iiii		: Deferential station's ID
		 *CC		: checksum for the sentence
	      */

	      pChar = stringRead;

	      /* Get UTC time */
	      j = 7;  /* start of time field */
	      k = 0;
	      while(*(pChar+j) != COMMA) {
		   timeString[k] = *(pChar+j);
		   j++;
		   k++;
	      }
	      lastCommaPosition = j;
	      timeString[k] = '\0';
	      sscanf(timeString, "%lu", &utcTime);
	      utcHour = (utcTime/10000);   /* extract Hours from long */
	      utcMinutes = (utcTime - (utcHour*10000))/100;  /* extract minutes from long */
	      utcSeconds = utcTime - (utcHour*10000) - (utcMinutes*100); /* extract seconds from long */
	     
	      /* NB: %02ld formats long to print 2 chars wide, padding with 0 if necessary */
	      printf("\t %02ld:%02ld:%02ld UTC \n", utcHour, utcMinutes, utcSeconds);

	      /* Get lattitude: ddmm.mmmm */
	      pChar = stringRead;
	      j = lastCommaPosition + 1;
	      k = 0;
	      while(*(pChar+j) != COMMA) {
		   latitudeString[k] = *(pChar+j);
		   j++;
		   k++;
	      }
	      lastCommaPosition = j;
	      latitudeString[k] = '\0';

	      sscanf(latitudeString, "%f", &latitude);
	      latDegrees = (int)(latitude/100);
	      latMinutes = (float)(latitude - latDegrees*100);
	      printf("\t %02d DEG  %2.4f MIN", latDegrees, latMinutes);

	      /* Get lattitude Cardinal direction */
	      pChar = stringRead;
	      j = lastCommaPosition + 1;
	      k = 0;
	      while(*(pChar+j) != COMMA) {
		   latitudeCardinalString[k] = *(pChar+j);
		   j++;
		   k++;
	      }
	      lastCommaPosition = j;
	      latitudeCardinalString[k] = '\0';
	      printf(" %s \n", latitudeCardinalString);

	      /* Get longitude: dddmm.mmmm */
	      pChar = stringRead;
	      j = lastCommaPosition + 1;
	      k = 0;
	      while(*(pChar+j) != COMMA) {
		   longitudeString[k] = *(pChar+j);
		   j++;
		   k++;
	      }
	      lastCommaPosition = j;
	      longitudeString[k] = '\0';

	      sscanf(longitudeString, "%f", &longitude);
	      longDegrees = (int)(longitude/100);
	      longMinutes = (float)(longitude - longDegrees*100);
	      printf("\t %03d DEG  %2.4f MIN", longDegrees, longMinutes);

	      printf("\n");

     // altitude
              
              pChar = stringRead;
	      j = lastCommaPosition + 12;
	      k = 0;
	      while(*(pChar+j) != COMMA) {
		   altitudeString[k] = *(pChar+j);
		   j++;
		   k++;
	      }
	      lastCommaPosition = j;
	      altitudeString[k] = '\0';
              sscanf(altitudeString, "%f", &altitude);                 
              printf("\n %f alti \n",altitude);            

                                                  

      //after all calculation in gps loop make a=0, jump from gps loop,get new imu, then a=1, delay(1s), while(a==1)


	  } /* else not a GPGGA sentence */

 //fprintf(gpsFile, "%d: (%d) %s  %flongitu: %flatiude: %faltiude:/n", numLinesRead, i, stringRead, longitude,latitude,altitude);

      } /* otherwise not a $ character... so loop back until one arrives */
 aa=0; 
 } while(aa == 1);

  printf("Exiting gps......");
  //close_com();   /* Finished with serial port so close it */
  fclose(gpsFile);
  printf("done\n");
  //latitude=12.34;
  //longitude=77.34;
  //utcTime=14.35;
  gps_v[0]=latDegrees+(latMinutes/60.0);
  gps_v[1]=longDegrees+(longMinutes/60.0);
  gps_v[2]=utcHour+(utcMinutes/60.0)+(utcSeconds/3600.0);
  gps_v[3]=altitude;
//printf("\n lat: %f long: %f utc: %f \n",latitude,longitude,utcTime);
//printf("\n lati: %f longit: %f utct: %f \n",gps_v[0],gps_v[1],gps_v[2]);
  serialClose(fd);
  return (0);
}
void rotate_az(int step_az, float speed_az)
{ 
  //rotate a specific number of degrees (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (step_az > 0)? 1:0;
  digitalWrite(pin_dir_az,dir); 
printf("in az function \n");
  step_az=abs(step_az);

  float usDelay = (1/speed_az) * 70;

  for(i=0; i < step_az; i++)
{ 
    digitalWrite(pin_cntrl_az,1); 
    delayMicroseconds(usDelay); 

    digitalWrite(pin_cntrl_az,0); 
    delayMicroseconds(usDelay); 
  } 
}
void rotate_al(int step_al, float speed_al)
{ 
  //rotate a specific number of degrees (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (step_al > 0)? 1:0;
  digitalWrite(pin_dir_al,dir); 

  step_al=abs(step_al);

  float usDelay = (1/speed_al) * 70;

  for(i=0; i < step_al; i++)
{ 
    digitalWrite(pin_cntrl_al,1); 
    delayMicroseconds(usDelay); 

    digitalWrite(pin_cntrl_al,0); 
    delayMicroseconds(usDelay); 
  } 
}
 //error- wiringPiSetup: Unable to open /dev/mem: Too many open files
