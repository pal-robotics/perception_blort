//CPP:
//			Title:			class tgMatrix3
//			File:				tgMatrix3.cpp
//
//			Function:		tgMatrix3 with all necessary operations 
//
//			Author:			Thomas M�rwald
//			Date:				20.09.2007
// ----------------------------------------------------------------------------
#include <blort/TomGine/tgMatrix3.h>

using namespace TomGine;

tgMatrix3::tgMatrix3(){
	for(int i=0; i<9; i++)
		m[i] = 0.0f;
}

tgMatrix3::tgMatrix3(tgVector3 x, tgVector3 y, tgVector3 z){
	m[0] = x.x;	m[1] = y.x;	m[2] = z.x;
	m[3] = x.y; m[4] = y.y; m[5] = z.y;
	m[6] = x.z; m[7] = y.z; m[8] = z.z;
}

tgMatrix3::tgMatrix3(const tgMatrix3 &m1){
	m[0]=m1.m[0];	m[1]=m1.m[1];	m[2]=m1.m[2];
	m[3]=m1.m[3];	m[4]=m1.m[4];	m[5]=m1.m[5];
	m[6]=m1.m[6];	m[7]=m1.m[7];	m[8]=m1.m[8];
}

tgMatrix3::tgMatrix3(	float m0, float m1, float m2,
					float m3, float m4, float m5,
					float m6, float m7, float m8){
	m[0]=m0; m[1]=m1; m[2]=m2;
	m[3]=m3; m[4]=m4; m[5]=m5;
	m[6]=m6; m[7]=m7; m[8]=m8;

}

tgMatrix3 tgMatrix3::operator+(tgMatrix3 m1){
	tgMatrix3 m2;
	return m2;
}

tgMatrix3 tgMatrix3::operator-(tgMatrix3 m1){
	tgMatrix3 m2;
	return m2;
}

tgMatrix3 tgMatrix3::operator*(tgMatrix3 m1){
	tgMatrix3 m2;
	
	m2.m[0] = m[0]*m1.m[0]	+	m[1]*m1.m[3]	+	m[2]*m1.m[6];
	m2.m[1] = m[0]*m1.m[1]	+	m[1]*m1.m[4]	+	m[2]*m1.m[7];
	m2.m[2] = m[0]*m1.m[2]	+	m[1]*m1.m[5]	+	m[2]*m1.m[8];
	
	m2.m[3] = m[3]*m1.m[0]	+	m[4]*m1.m[3]	+	m[5]*m1.m[6];
	m2.m[4] = m[3]*m1.m[1]	+	m[4]*m1.m[4]	+	m[5]*m1.m[7];
	m2.m[5] = m[3]*m1.m[2]	+	m[4]*m1.m[5]	+	m[5]*m1.m[8];
	
	m2.m[6] = m[6]*m1.m[0]	+	m[7]*m1.m[3]	+	m[8]*m1.m[6];
	m2.m[7] = m[6]*m1.m[1]	+	m[7]*m1.m[4]	+	m[8]*m1.m[7];
	m2.m[8] = m[6]*m1.m[2]	+	m[7]*m1.m[5]	+	m[8]*m1.m[8];
	
	return m2;	
}

tgMatrix3 tgMatrix3::operator*(float f){
	tgMatrix3 m2;
	return m2;
}

tgVector3 tgMatrix3::operator*(tgVector3 v1){
	tgVector3 v2;
	
	v2.x = m[0]*v1.x + m[1]*v1.y + m[2]*v1.z;
	v2.y = m[3]*v1.x + m[4]*v1.y + m[5]*v1.z;
	v2.z = m[6]*v1.x + m[7]*v1.y + m[8]*v1.z;
	
	return v2;
}

void tgMatrix3::transpose(){
	tgMatrix3 m1 = tgMatrix3(*this);
	
	m[0]=m1.m[0];	m[1]=m1.m[3];	m[2]=m1.m[6];
	m[3]=m1.m[1];	m[4]=m1.m[4];	m[5]=m1.m[7];
	m[6]=m1.m[2];	m[7]=m1.m[5];	m[8]=m1.m[8];
}



