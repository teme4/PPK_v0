
class gpio
{
private: 


 void Set_pin_H(GPIO_TypeDef *port,uint8_t pin);
 void Set_pin_L(GPIO_TypeDef *port,uint8_t pin);

public:

 //            General purpose output
uint8_t gpio_mode_pp_2=             0x01;
uint8_t gpio_mode_pp_10=            0x02;
uint8_t gpio_mode_pp_50=            0x03;
uint8_t gpio_mode_od_2=             0x05;
uint8_t gpio_mode_od_10=    		    0x06;
uint8_t gpio_mode_od_50=		      	0x07;
//             Alternate Function output
uint8_t alternate_mode_pp_2=	    	0x09;
uint8_t alternate_mode_pp_10=      	0x0A;
uint8_t alternate_mode_pp_50=		    0x0B;
uint8_t alternate_mode_od_2=	    	0x0D;
uint8_t alternate_mode_od_10=		    0x0E;
uint8_t alternate_mode_od_50=		    0x0F;
//                Input
uint8_t input_mode_analog=          0x00;
uint8_t input_mode_floating=	      0x04;
uint8_t input_mode_pull_down=       0x28;  
uint8_t input_mode_pull_up=         0x48;//0x80



  void gpio_conf(GPIO_TypeDef * port, uint32_t pin, uint8_t mode);
  void set_pin_state(GPIO_TypeDef *GPIOx,uint8_t pin,uint8_t state);
  int get_state_pin (GPIO_TypeDef *GPIOx, uint8_t pin);
  //void config_af(GPIO_TypeDef *GPIOx, uint8_t PIN, uint8_t AF); 
};


