#include "main.h"

// NEPOUZIVAT PINY B6 A B7, JSOU VYHRAZENY PRO SERIOVOU KOMUNIKACI
// BAUD RATE = 9600
// ========================== PRIPRAVA DAT A VYSTUPU ===========================
// pomocne konstanty
#define  LEFT  0
#define  RIGHT 1
#define  DET_EN   0                       // povoluje nebo zakazuje vyhodnoceni SHARP

// regulator
#define  CONP  2                          // konstanta pro proporcionalni regulator (2)
#define  CONI  0                         // konstanta pro integracni regulator *0,01  (45)
#define  COND  0                         // konstanta pro derivacni regulator *0,01   (20)

#define  SPD_LO   220                     // zaklad pro vypocet rychlosti pomalejsiho motoru (130)
#define  SPD_HI   220                     // zaklad pro vypocetrychlosti rychlejsiho motoru  (155)
#define  SPD_MAX  220                     // rychlost po rovince  (240)

int8 reg_out;
int8 err1;                                // odchylka prvni hodnoty
int8 err2;                                
int8 err3;
int8 err4;
int8 err5;                                // odchylka posledni hodnoty
int8 errp;                                // prumer chyb

// mezera
#define SPACE  8                          // jak dlouho robot smi nic nevidet (8)
#define CONT   25                         // kontrast, kdy nic nevidime

// univerzalni LED diody
#define  LED1     PIN_E1
#define  LED2     PIN_E0

int16 blink;

// piezo pipak
#DEFINE SOUND_HI   PIN_B4
#DEFINE SOUND_LO   PIN_B5

// radkovy senzor
#define  SDIN     PIN_D4                  // seriovy vstup
#define  SDOUT    input(PIN_C5)           // seriovy vystup
#define  SCLK     PIN_D5                  // takt

// pro komunikaci s OLSA, prvni se posila LSB
int MAIN_RESET[8]={1,1,0,1,1,0,0,0};      // hlavni reset 0x1B
int SET_MODE_RG[8]={1,1,1,1,1,0,1,0};     // zapis do MODE registru 0x5F
int CLEAR_MODE_RG[8]={0,0,0,0,0,0,0,0};   // nulovani MODE registru 0x00

int LEFT_OFFSET[8]={0,0,0,0,0,0,1,0};     // offset leveho segmentu senzoru 0x40
int MID_OFFSET[8]={0,1,0,0,0,0,1,0};      // offset prostredniho segmentu senzoru 0x42
int RIGHT_OFFSET[8]={0,0,1,0,0,0,1,0};    // offset praveho segmentu senzoru 0x44
int OFFSET[8]={1,0,0,0,0,0,0,1};          // minus jedna - pouzit pro vsechny segmenty 0x81

int LEFT_GAIN[8]={1,0,0,0,0,0,1,0};       // zisk leveho segmentu 0x41
int MID_GAIN[8]={1,1,0,0,0,0,1,0};        // zisk leveho segmentu 0x43
int RIGHT_GAIN[8]={1,0,1,0,0,0,1,0};      // zisk leveho segmentu 0x45
int GAIN[8]={1,0,1,0,0,0,0,0};            // zisk = 5 - pouzit pro vsechny segmenty 0x5

int START_INT[8]={0,0,0,1,0,0,0,0};       // zacatek integrace 0x08
int STOP_INT[8]={0,0,0,0,1,0,0,0};        // konec integrace 0x10
int READOUT[8]={0,1,0,0,0,0,0,0};         // cteni senzoru 0x02

int olsa_lseg[51]={0};                    // leva cast radky (pixely 0 - 50)
int olsa_rseg[51]={0};                    // prava cast radky (pixely 51 - 101)
int8  *lp;                                // ukazatel pro levou polovinu radky
int8  *rp;                                // ukazatel pro levou polovinu radky

int8  contrast;                           // rozdil mezi nejsvetlejsim a nejtmavsi mistem
int8  position;                           // ulozeni pozice cary
int8  old_position;                       // ulozeni predchozi pozice cary
int1  line_sector;                        // cara je vlevo/vpravo
int8  gap;                                // pocita, jak dlouho neni videt cara

// ================================= NARAZNIK ==================================
#define  BUMPL    input(PIN_D6)
#define  BUMPR    input(PIN_D7)

// ============================= NOUZOVE SENZORY ===============================
#define  LINEL    0                       // analogovy kanal pro levy senzor
#define  LINER    1                       // analogovy kanal pro pravy senzor
#define  WHITE    30                      // rozhodovaci uroven pro nouzove senzory

int8  line_l;                             // uklada hodnotu leveho senzoru
int8  line_r;                             // uklada hodnotu praveho senzoru

// ================================ DALKOMER ===================================
#define  SHARP    2                       // analogovy kanal pro SHARP
#define  PROBLEM  55                      // rozhodovaci uroven, kdy hrozi prekazka
#define  BLOCK    65                      // rozhodovaci uroven, kdy je jiste prekazka
#define  DANGER   10                      // pocita, jak dlouho je detekovan problem

int8  p_count;
int8  sharp_lev;                          // uklada hodnotu sharp

// ================================== MOTORY ===================================
#define  LMF   PIN_D0
#define  LMB   PIN_D1
#define  RMF   PIN_D2
#define  RMB   PIN_D3

int8  lm_speed;                           // rychlost leveho motoru
int8  rm_speed;                           // rychlost praveho motoru

// =============================== PODPROGRAMY =================================

// ================================= OLSA01A ===================================

void  olsa_pulses(int count)              // vytvori impulzy pro ridici logiku
{
   int8 ct;
   for(ct=0;ct<=count;ct++)
   {
      output_high(SCLK);
      output_low(SCLK);
   }
} 

void olsa_pulse()                         // vytvori jeden impulz
{
   output_high(SCLK);
   output_low(SCLK);
}

void olsa_send(int8 info[8])              // USART komunikace s modulem OLSA01A - poslani zpravy
{
   int *ip;                               // ukazatel na pole s informaci
   int8 i;                                // pomocna promenna pro nastaveni 0 nebo 1 na SDIN
   output_low(SDIN);                      // start bit
   olsa_pulse();
   for(ip=0;ip<8;ip++)                    // predani informace - 8 bit, LSB prvni > MSB posledni
   {
      i=info[ip];                         // ziskani hodnoty z pole
      if(i==1)                            // vyhodnoceni obsahu informace - nastav 1
      {
         output_high(SDIN);
      }
      else                                // vyhodnoceni obsahu informace - nastav 0
      {
         output_low(SDIN);
      }
      olsa_pulse();   
   }
   output_high(SDIN);                     // stop bit
   olsa_pulse();
}

void olsa_reset()                         // hlavni RESET - provadi se po zapnuti
{
   output_low(SDIN);
   output_low(SCLK);
   olsa_pulses(30);                       // reset radkoveho senzoru
   output_high(SDIN);
   olsa_pulses(10);                       // start bit - synchronizace
   olsa_send(MAIN_RESET);
   olsa_pulses(5);
   olsa_send(SET_MODE_RG);
   olsa_send(CLEAR_MODE_RG);
}
   
void olsa_setup()                         // kompletni nastaveni, provadi se po resetu
{
   olsa_send(LEFT_OFFSET);                // nastaveni leveho segmentu (offset a zisk)
   olsa_send(OFFSET);
   olsa_send(LEFT_GAIN);
   olsa_send(GAIN);
   olsa_send(MID_OFFSET);                 // nastaveni prostredniho segmentu (offset a zisk)
   olsa_send(OFFSET);
   olsa_send(MID_GAIN);
   olsa_send(GAIN); 
   olsa_send(RIGHT_OFFSET);               // nastaveni praveho segmentu (offset a zisk)
   olsa_send(OFFSET);
   olsa_send(RIGHT_GAIN);
   olsa_send(GAIN); 
}
   
void olsa_integration()                   // snimani pixelu
{
   olsa_send(START_INT);                  // zacatek integrace senzoru
   olsa_pulses(22);
   olsa_send(STOP_INT);                   // konec integrace senzoru
   olsa_pulses(5);
}

void read_olsa()
{
   int8  cpixel;                          // pocet prectenych pixelu
   int8  cbit;                            // pocet prectenych bitu
   int8  pixel;                           // hodnota precteneho pixelu      
   cpixel=0;
   lp=0;
   rp=0;
   olsa_integration();
   olsa_send(READOUT);
   do                                     // precte 102 pixelu
   {
      if(!SDOUT)                          // zacatek prenosu - zachycen start bit
      {  
         pixel=0;
         for(cbit=0;cbit<8;cbit++)        // cte jednotlive bity (8 bitu - 0 az 7)         
         {
            olsa_pulse();                 // impulz pro generovani dalsiho bitu
            
            if(SDOUT)                     // zachycena 1                        
            {
               bit_set(pixel,cbit);       // zapise do bitu (dano cbit) bytu (pixelu) prislusnou hodnotu
            }
         }
         olsa_pulse();                    // generuje stop bit
         if(cpixel<52)                    // ulozeni do pole
         {
            olsa_lseg[lp]=pixel;          // leva polovina radky - leve pole
            lp++;
         }
         else
         {
            olsa_rseg[rp]=pixel;          // prava polovina cary - prave pole
            rp++;
         }
         cpixel++;
      }
      else
      {
         olsa_pulse();                    // generuje start bit, nebyl-li poslan
      }
   }
   while(cpixel<102);                     // precte 102 pixelu
}

void olsa_position()                      // vyhodnoti pozici cary
{
   int8  searchp;                         // ukazatel na pole
   int8  dark;                            // nejtmavsi pixel
   int8  bright;                          // nejsvetlejsi pixel
   dark=0xff; 
   bright=0x00;
   for(searchp=0;searchp<51;searchp++)    // prohlizi levou cast radky
   {      
      if(olsa_lseg[searchp]<dark)         // porovna pixel s doposud nejtmavsim
      {
         dark=olsa_lseg[searchp];         // ulozi nejtmavsi pixel
         position=searchp;                // ulozi polohu nejtmavsiho pixelu
      }
      if(olsa_lseg[searchp]>bright)
      {
         bright=olsa_lseg[searchp];       // ulozi nejsvetlejsi pixel
      }
   }
   for(searchp=0;searchp<49;searchp++)    // prohlizi levou cast radky
   {      
      if(olsa_rseg[searchp]<dark)         // porovna pixel s doposud nejtmavsim
      {
         dark=olsa_rseg[searchp];         // ulozi nejtmavsi pixel
         position=(searchp+51);           // ulozi polohu nejtmavsiho pixelu
      }
      if(olsa_rseg[searchp]>bright)
      {
         bright=olsa_rseg[searchp];       // ulozi nejsvetlejsi pixel
      }
   }
   contrast=(bright-dark);
   if(contrast<CONT)
   {
      position=0;  
   }
}

// ============================ ZACHRANNE SENZORY ==============================

void read_blue_sensors()                        // cteni nouzovych senzoru
{
   set_adc_channel(LINEL);                      // cti levy nouzovy senzor
   delay_us(10);
   line_l=read_adc();     
   set_adc_channel(LINER);                      // cti pravy nouzovy senzor
   delay_us(10);
   line_r=read_adc();
}

// ================================= DALKOMER =================================

void read_sharp()                               // cteni z dalkomeru
{
   set_adc_channel(SHARP);                      // cteni z dalkomeru                      
   delay_us(10);
   sharp_lev=read_adc();                        // ulozeni hodnoty
}
   
// ================================== PIPAK ====================================

void beep(int16 period,int16 length)
{
   int16 bp;                                    // promenna pro nastaveni delky
   for(bp=length;bp>0;bp--)                     // prepina vystupy tolikrat, jakou jsme zadali delku
   {
      output_high(SOUND_HI);
      output_low(SOUND_LO);
      delay_us(period);
      output_high(SOUND_LO);
      output_low(SOUND_HI);
      delay_us(period);
   }
}

// ================================= REGULATOR =================================

void calc_error()
{
   err1=err2;                                // ulozeni chyb
   err2=err3;
   err3=err4;
   err4=err5;
   if(position<50)                           // ulozeni a vypocet aktualni absolutni chyby
   {
      err5=(50-position);                    // pokud je cara vlevo
   }
   else
   {
      err5=(position-50);                    // pokud je cara vpravo
   }
   errp=((err1+err2+err3+err4+err5)/5);      // vypocet chyby pro integracni regulator
}
void calc_regulator()
{
   int8  p_reg;
   int8  i_reg;
   int8  d_reg;
   p_reg=(CONP*err5);                                 // vypocet proporcionalni slozky
   i_reg=(CONI*(errp/100));                           // vypocet integracni slozky
   if(position>old_position)                          // vypocet derivacni slozky
   {
      d_reg=(COND*((position-old_position)/100));     // pokud je aktualni pozice vetsi nez predesla
   }
   else
   {
      d_reg=(COND*((old_position-position)/100));     // pokud je aktualni pozice mensi nez predesla
   }
   reg_out=(p_reg+i_reg+d_reg);                       // vypocet celeho regulatoru
}

// ================================== MOTORY ===================================

void l_motor_fwd(int8 speedl)                   // levy motor dopredu
{
   output_high(LMF);
   output_low(LMB);
   set_pwm2_duty(speedl);
}

void l_motor_bwd(int8 speedl)                   // levy motor dozadu
{
   output_high(LMB);
   output_low(LMF);
   set_pwm2_duty(speedl);
}

void r_motor_fwd(int8 speedr)                   // pravy motor dopredu
{
   output_high(RMF);
   output_low(RMB);
   set_pwm1_duty(speedr);
}

void r_motor_bwd(int8 speedr)                   // pravy motor dozadu
{
   output_high(RMB);
   output_low(RMF);
   set_pwm1_duty(speedr);
}

void l_motor_off()                              // levy motor vypnut
{
   output_low(LMF);
   output_low(LMB);
   set_pwm2_duty(0);
}
   
void r_motor_off()                              // pravy motor vypnut
{  
   output_low(RMF);
   output_low(RMB);
   set_pwm1_duty(0);
}

void motor_test()                               // TEST MOTORU
{
   int8  i;
   beep(100,200);
   printf("TEST MOTORU\r\n");
   delay_ms(1000);
   printf("LEVY MOTOR DOPREDU\r\n");
   delay_ms(1000);
   for(i=0;i<255;i++)                           // levy motor dopredu - zrychluje
   {
      l_motor_fwd(i);
      printf("RYCHLOST: %u\r\n",i);
      delay_ms(5);
   }
   for(i=255;i>0;i--)                           // levy motor dopredu - zpomaluje
   {
      l_motor_fwd(i);
      printf("RYCHLOST: %u\r\n",i);
      delay_ms(5);
   }   
   printf("LEVY MOTOR DOZADU\r\n");             // levy motor dozadu - zrychluje
   delay_ms(1000);
   for(i=0;i<255;i++)
   {
      l_motor_bwd(i);
      printf("RYCHLOST: %u\r\n",i);
      delay_ms(5);
   }   
   for(i=255;i>0;i--)                           // levy motor dozadu - zpomaluje
   {
      l_motor_bwd(i);
      printf("RYCHLOST: %u\r\n",i);
      delay_ms(5);
   }   
   printf("PRAVY MOTOR DOPREDU\r\n");     
   delay_ms(1000);
   for(i=0;i<255;i++)                           // pravy motor dopredu - zrychluje
   {
      r_motor_fwd(i);
      printf("RYCHLOST: %u\r\n",i);
      delay_ms(5);
   }   
   for(i=255;i>0;i--)                           // pravy motor dopredu - zpomaluje
   {
      r_motor_fwd(i);
      printf("RYCHLOST: %u\r\n",i);
      delay_ms(5);
   }   
   printf("PRAVY MOTOR DOZADU\r\n");
   delay_ms(1000);
   for(i=0;i<255;i++)                           // pravy motor dozadu - zrychluje
   {
      r_motor_bwd(i);
      printf("RYCHLOST: %u\r\n",i);
      delay_ms(5);
   }   
   for(i=255;i>0;i--)                           // pravy motor dozadu - zpomaluje
   {
      r_motor_bwd(i);
      printf("RYCHLOST: %u\r\n",i);
      delay_ms(5);
   }
   l_motor_off();                               // po ukonceni testu vypnout motory    
   r_motor_off();
   printf("KONEC TESTU MOTORU\r\n");      
   delay_ms(1000);
}

// ================================ OBJETI CIHLY ===============================

void detour()                                   // po detekci prekazky zacne objizdeni
{
   l_motor_bwd(200);                            // zatoc doleva
   r_motor_fwd(200);
   delay_ms(400);
   l_motor_fwd(200);                            // jed rovne
   delay_ms(800);
   r_motor_bwd(200);                            // zatoc doprava
   delay_ms(200);
   r_motor_fwd(200);                            // jed rovne
   delay_ms(1000);
   r_motor_bwd(200);                            // zatoc doprava
   delay_ms(100);
   l_motor_fwd(180);
   r_motor_fwd(255);                            // jed rovne
   delay_ms(200);
   position=40;
}
// ================================ DIAGNOSTIKA ================================

void diag()                                     // diagnostika - vypis senzoru s moznosti prepnuti na test motoru
{
   read_blue_sensors();                         // cteni nouzovych senzoru
   read_sharp();                                // cteni dalkomeru
   read_olsa();                                 // cteni z optickeho radkoveho senzoru
   olsa_position();
   printf("LEVA: %u \t",line_l);                // tiskne z leveho senzoru
   printf("PRAVA: %u \t",line_r);               // tiskne z praveho senzoru
   printf("SHARP: %u \t",sharp_lev);            // tiskne z dalkomeru
   printf("POLOHA: %u\t",position);             // tiskne pozici OLSA 
   printf("KONTRAST: %u \t", contrast);         // tiskne kontrast z OLSA
   printf("L_NARAZ: %u \t",BUMPL);              // leve tlacitko narazniku
   printf("P_NARAZ: %u \r\n",BUMPR);            // prave tlacitko narazniku
   if(BUMPL&&BUMPR)                             // po zmacknuti stran narazniku spusti test motoru
   {
      beep(100,1000);
      printf("Levy naraznik - test OLSA\r\n");
      printf("Pravy naraznik - test motoru\r\n");
      delay_ms(500);
      while(true)
      {
         if(BUMPR)
         {
            beep(100,500);                      // pipni pri startu
            motor_test();
         }
         if(BUMPL)
         {
            beep(100,500);
            printf("TEST OLSA\r\n");
            while(true)
            {
               int8  tisk;
               int8  *tiskp;
               read_olsa();
               printf("cteni\r\n");             // po precteni vsech pixelu odradkuje
               for(tiskp=0;tiskp<52;tiskp++)    // tisk leve casti radky
               {
                  tisk=olsa_lseg[tiskp];
                  printf("%x ",tisk);
               }
               for(tiskp=0;tiskp<52;tiskp++)    // tisk prave casti radky
               {
                  tisk=olsa_rseg[tiskp];
                  printf("%x ",tisk);
               }
            }
         }     
      }
   }   
}

// ============================== HLAVNI SMYCKA ================================

void main()
{
   printf("POWER ON \r\n");
   // NASTAVENI > provede se pouze pri zapnuti       
   setup_adc(ADC_CLOCK_INTERNAL);                     // interni hodniny pro AD prevodnik
   setup_adc_ports(ALL_ANALOG);                       // aktivni vsechny analogove vstupy
   setup_spi(SPI_SS_DISABLED);
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_1);
   setup_timer_1(T1_DISABLED);
   setup_timer_2(T2_DIV_BY_16,255,1);                 // casovac pro PWM
   setup_ccp1(CCP_PWM);                               // povoli PWM na pinu RC2
   setup_ccp2(CCP_PWM);                               // povolí PWM na pinu RC1
   setup_comparator(NC_NC_NC_NC);
   setup_vref(FALSE);
   l_motor_off();                                     // vypne levy motor
   r_motor_off();                                     // vypne pravy motor
   olsa_reset();
   olsa_setup();
   beep(350,300);                                     // pipni pri startu
   printf("OK! \r\n");
   delay_ms(500);
   printf("VYBRAT MOD... \r\n");
// ============================ HLAVNI CAST PROGRAMU ===========================
   while(true)
   {
      if(blink<4000)
      {
         output_low(LED1);
         output_high(LED2);
      }
      else
      {
         output_low(LED2);
         output_high(LED1);
      }
      if (blink==8000)
      {
         blink=0;
      }
      blink++;
// ================================ DIAGNOSTIKA ================================  
      if(BUMPL)
      {
         output_low(LED1);
         output_high(LED2);
         beep(200,500);
         while(true)
         {
            diag();
         }
      }
// =============================== SLEDOVANI CARY ==============================
      if(BUMPR)                                       // spusteni hledani pravym naraznikem
      { 
         output_low(LED2);
         output_high(LED1);
         beep(300,500);
         while(true)
         {
            old_position=position;                    // zaznamena predhozi polohu cary
            read_olsa();                              // precte a ulozi hodnoty z olsa
            olsa_position();                          // vyhodnoti pozici cary
            read_blue_sensors();                      // cte nouzove senzory
            read_sharp();                             // cte dalkomer
            if(position==0)                           // pokud neni videt cara
            {
               position=old_position;                 // nastav predchozi pozici
               gap++;                                 // pocita, jak dlouho neni videt cara
            }
            else                                      // pokud je videt
            {
               gap=0;                                 // gap je roven nule
            }
            if(gap>SPACE)
            {
               if(line_l<WHITE)                       // cara videna levym modrym senzorem
               {
                  position=1;
                  line_sector=LEFT;
               }
               if(line_r<WHITE)                       // cara videna pravym modrym senzorem
               {
                  position=100;
                  line_sector=RIGHT;
               }
            }
            calc_error();
            calc_regulator();
            //printf("regulator: %u\r\n",reg_out);
            if(position<47)                           // prepocet regulatoru pro motory, pokud je cara vlevo
            {
               lm_speed=SPD_LO-(2*reg_out);
               rm_speed=SPD_HI;
               line_sector=LEFT;
            }
            if((position>46)&&(position<54))          // nastaveni rychlosti, pokud je cara uprostred
            {
               lm_speed=SPD_MAX;
               rm_speed=SPD_MAX;
            }
            if(position>53)                           // prepocet regulatoru pro motory, pokud je cara vpravo
            {
               lm_speed=SPD_HI;
               rm_speed=SPD_LO-(2*reg_out);
               line_sector=RIGHT;
            }
            if((sharp_lev>PROBLEM)&&(DET_EN))         // zachycen odraz na dalkomeru
            {
               p_count++;                             // pocita, jak dlouho je videt
               output_low(LED1);
               if(p_count>DANGER)                     // pokud je odraz videt nebezpecne dlouho, zpomali
               {
                  lm_speed=(lm_speed/2);
                  rm_speed=(rm_speed/2);
               }
            }
            else                                      // pokud jiz neni detekoven odraz, vynuluj pocitadlo
            {
               p_count=0;
               output_high(LED1);
            }
            if((sharp_lev>BLOCK)&&(DET_EN))           // odraz zachycen nebezpecne blizko
            {
               l_motor_off();
               r_motor_off();
               beep(100,200);
               read_sharp();
               if(sharp_lev>BLOCK)                    // pokud je porad videt prekazka
               {
                  detour();                           // spust objizdeni
               }
            }
            l_motor_fwd(lm_speed);
            r_motor_fwd(rm_speed);            
         }
      }
   }
}


