static PT_THREAD (protothread_FM(struct pt *pt))
{
  PT_BEGIN(pt);
    
    // convert alarm period in uSEc to rate
    Fs = 1.0/((float)alarm_period*1e-6) ;
    //
    while(1) {
      
      // == Fout and Fmod are in Hz
      // == fm_depth is 0 to 10 or so
      // == times are in seconds
      // wait for the run command
      PT_YIELD_UNTIL(pt, menu[10].item_float_value==1);

      // conversion to intrnal units
      // increment = Fout/Fs * 2^32
      // octave number is based on a C3 to C4 table
    
      octave_num = menu[0].item_float_value ;
      Fmod =  menu[4].item_float_value ;
      float current_note ;
      for(int i=0; i<8; i++){
        current_note = notes[i] * pow(2, octave_num-3) ;
        main_inc[i] = current_note * pow(2,32 )/ Fs ;
        mod_inc[i] = Fmod * current_note * pow(2,32 )/ Fs ;
      }
      // fm modulation strength
      max_mod_depth = float_to_fix(menu[5].item_float_value * 100000) ;

      // convert main input times to sample number
      attack_time = float_to_fix(menu[1].item_float_value * Fs) ;
      decay_time = float_to_fix(menu[3].item_float_value * Fs) ;
      sustain_time = float_to_fix(menu[2].item_float_value * Fs) ;
      // and now get increments
      attack_inc = div(max_amp, attack_time) ;
      // linear and parabolic fit
      decay_inc = div(max_amp, decay_time) ; 
      recip_decay_time = div(onefix, decay_time);
      //quad_decay_inc = mul((decay_inc<<1), recip_decay_time);
      // this need to be floating point becuase of squareing the decay time
      //f_quad_decay_inc = (fix_to_float(decay_inc<<1)/fix_to_float(decay_time));

      // convert modulation input times to sample number
      mod_attack_time = float_to_fix(menu[6].item_float_value * Fs) ;
      mod_decay_time = float_to_fix(menu[8].item_float_value * Fs) ;
      mod_sustain_time = float_to_fix(menu[7].item_float_value * Fs) ;
      // and now get increments
      // precomputing increments means that only add/subtract is needed
      mod_attack_inc = div(max_mod_depth, mod_attack_time) ;
      mod_decay_inc = div(max_mod_depth, mod_decay_time) ;
      //
      //printf("start\n\r") ;
      // tell the synth ISR to go
      static int i, j ;

        //mod_attack_inc = div(current_depth, mod_attack_time) ;
        //mod_decay_inc = div(current_depth, mod_decay_time) ;
        for(i=0; i<8; i++){
            current_main_inc = main_inc[i] ;
            current_mod_inc = mod_inc[i] ;
            note_start = true ;
            PT_YIELD_usec(20000) ;
            PT_YIELD_UNTIL(pt, current_amp<onefix);
            PT_YIELD_usec(10000) ;
        }

      // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // timer thread


void compute_sample(void)
{
    // === 
    // start a burst on new data
    if(note_start) {
        // init the amplitude
        current_amp = attack_inc ;
        current_mod_depth = mod_attack_inc ;
        // reset the start flag
        note_start = false ;
        // reset envelope time
        note_time = 0 ;
        // phase lock the main frequency
        main_accum = 0 ;
    } // note start
    // play the burst as long as the amplitude is positive
    // as it decays linearly
    if (current_amp > 0) {

        // update dds modulation freq
        mod_accum += current_mod_inc ;
        mod_wave = sine_table[mod_accum>>24] ;
        // get the instataneous modulation amp 
        // update modulation amplitude envelope 
        if (note_time < (mod_attack_time + mod_decay_time + mod_sustain_time)){
            current_mod_depth = (note_time <= mod_attack_time)? 
                current_mod_depth + mod_attack_inc : 
                (note_time <= mod_attack_time + mod_sustain_time)? current_mod_depth:
                    current_mod_depth - mod_decay_inc ;
        }
        else { 
            current_mod_depth = 0 ;
        }

        // set dds main freq and FM modulate it
        main_accum += current_main_inc + (unsigned int) mul(mod_wave, current_mod_depth) ;
        // update main waveform
        main_wave = sine_table[main_accum>>24] ;

        // get the instataneous amp 
        // update amplitude envelope 
        // linear EXCEPT for optional parabolic decay
        if (note_time < (attack_time + decay_time + sustain_time)){
            if (note_time <= attack_time) current_amp += attack_inc ;
            else if (note_time > attack_time + sustain_time){
                if (linear_dk==1) {current_amp -= decay_inc ;}
                else {current_amp = current_amp - (decay_inc<<1) + 
                        div(mul((decay_inc<<1), (note_time-attack_time-sustain_time)), decay_time) ;
                }
            }
        }
        else {
            current_amp = 0 ;
        }
        // amplitide modulate and shift to the correct range for PWM
        main_wave = mul(main_wave, current_amp) ;
        // write final result  to 8-bit PWM    
        //pwm_set_chan_level(pwm_slice_num, pwm_chan_num, fix_to_int(main_wave) + 128) ;
        //pwm_set_chan_level(pwm_slice_num, pwm_chan_num, fix_to_int(current_amp)) ;
        DAC_data = (DAC_config_chan_A | ((fix_to_int(main_wave) + 2048) & 0xfff))  ;

        // Write data to DAC
        spi_write16_blocking(SPI_PORT, &DAC_data, 1) ;
        // move time ahead
        note_time += onefix ;
    } // current amp > 0
    else {
        // set PWM to neutral level
        DAC_data = (DAC_config_chan_A | ((2048) & 0xfff))  ;
    }   

    // save in buffer for FFT
    if(buffer_index < N_WAVE){
        dds[buffer_index++] = dds_to_s1x14(main_wave) ;
    }
} // end ISR call



