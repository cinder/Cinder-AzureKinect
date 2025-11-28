#version 430

uniform sampler2DRect	uTexColorBuffer;

in vec4	vColor;
in vec2 vTexCoord;

out vec4 oFragColor;

void main()
{
    oFragColor = vec4( texture( uTexColorBuffer, vTexCoord ).rgb, 1.0 );
}