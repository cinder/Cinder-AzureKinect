#version 150

uniform mat4 	ciModelViewProjection;

uniform sampler2DRect	uTexDepthBuffer;
uniform sampler2DRect	uDepthTable;

in vec2 ciPosition; // x, y index into buffers
in vec4 ciColor;

out vec4 vColor;
out vec2 vTexCoord;

void main()
{	
	vColor = ciColor;
	vTexCoord = ciPosition.xy;

	vec3 vertPos;
	float depthMillimeters = texture( uTexDepthBuffer, ciPosition.xy ).r * 65535.0;
	vec3 tableValue = texture( uDepthTable, ciPosition.xy ).xyz;
	if( depthMillimeters < 0.00001 || isnan(tableValue.x) )
		vertPos.xyz = vec3( 10e10 ); // cull off-screen
	else
		vertPos = tableValue * depthMillimeters;


	gl_Position = ciModelViewProjection * vec4( vertPos, 1.0 );
}