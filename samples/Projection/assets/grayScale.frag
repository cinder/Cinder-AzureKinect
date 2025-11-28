#version 150

uniform sampler2D uTex0;
uniform vec2 uRange;

in vec2		TexCoord;
out vec4 	oColor;

void main( void )
{
    float value = texture( uTex0, TexCoord.st ).r;
    if( value > 0.00001 ) { // keep true black true
        value = clamp( value, uRange.x, uRange.y );
        value = uRange.x + ( value - uRange.x ) / ( uRange.y - uRange.x );
        oColor = vec4( vec3( value ), 1.0 );
    }
    else
        oColor = vec4( 0, 0, 0, 1 );
}
