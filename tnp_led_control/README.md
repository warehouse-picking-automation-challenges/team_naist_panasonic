LED intensity controller

Please set LED intensity 000 ~ 255

//execute

roslaunch tnp_led_control LED_controller.launch

//set_led_intensity (example:050)

rosservice call /tnp_led_control/set_led_intensity "target_intensity:
  data: '050'" 
succeeded: True

//get_led_intensity  (Plese call this service after calling "set_led_intensity" )

rosservice call /tnp_led_control/get_led_intensity "{}"