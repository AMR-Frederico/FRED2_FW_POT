
#ifndef TOOLS // include guard
#define TOOLS


// -------------------------------------------------------
// Saturation for positive values
// -------------------------------------------------------

int saturation_under(int value, int saturation_value){
    if(value < saturation_value ){

        value = saturation_value;
    }

    return value ; 

}

// -------------------------------------------------------
// Saturation for positive and negative values
// -------------------------------------------------------

int saturation(int value,int saturation_value){
    if(value>saturation_value){
        value = saturation_value ;
    }
    else if( value < -saturation_value){
        value = -saturation_value ;
    }

    return value;


}
#endif