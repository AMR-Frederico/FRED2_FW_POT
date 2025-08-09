

#ifndef ENCODER_HPP // include guard
#define ENCODER_HPP
  
  #include <Arduino.h>
  #define LEFT   0
  #define RIGHT  1

  struct EncoderData{
    double angle;
    double rpm;
    double ticks;
  };

  class Encoder
  {
    public:
      Encoder(int pin_AL, int pin_BL, int pin_AR, int pin_BR);
      void setup();
      double readPulses(int encoder_side);
      double readAngle(int encoder_side);
      double readRPM(int encoder_side);
      EncoderData get_encoder_data(int id);
      void debugPrint();
      void reset();
      int DI_ENCODER_CH_AL;
      int DI_ENCODER_CH_BL;
      int DI_ENCODER_CH_AR;
      int DI_ENCODER_CH_BR;
        
    private:
      static void IRAM_ATTR interruptionChAL();
      static void IRAM_ATTR interruptionChBL();
      static void IRAM_ATTR interruptionChAR();
      static void IRAM_ATTR interruptionChBR();

      // objeto que representa a classe Encoder
      static Encoder* obj_Encoder; 
      // Encoder* obj_Encoder; 

  };

  
#endif