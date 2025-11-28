#version 150

uniform sampler2D uTex0;
uniform float uDepthMin, uDepthMax;

in vec2		TexCoord;
out vec4 	oColor;

vec3 hue2rgb( float hue )
{
    hue = fract( hue ); //only use fractional part of hue, making it loop
    hue *= 2.0 / 3.0;
    float r = abs(hue * 6.0 - 3.0) - 1.0;
    float g = 2 - abs(hue * 6.0 - 2.0);
    float b = 2 - abs(hue * 6.0 - 4.0);
    return clamp( vec3( r, g, b ), 0.0, 1.0 );
}

void main( void )
{
    highp float depth = texture( uTex0, TexCoord.st ).r;
    if( depth > 0.00001 ) { // keep true black true
        depth = clamp( depth, uDepthMin, uDepthMax );
        depth = uDepthMin + ( depth - uDepthMin ) / ( uDepthMax - uDepthMin );
        oColor = vec4( hue2rgb( 1.0 - depth ), 1.0 );
    }
    else
        oColor = vec4( 0, 0, 0, 1 );

   // oColor = vec4( vec3( depth ), 1.0 );
}
