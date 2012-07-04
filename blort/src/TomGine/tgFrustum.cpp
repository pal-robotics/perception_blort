
#include <blort/TomGine/tgFrustum.h>

using namespace TomGine;

void tgFrustum::ExtractFrustum(){
	float   clip[16];
	float   t;
	float mv[16];
	float pj[16];
 
	/* Get the current PROJECTION matrix from OpenGL */
	glGetFloatv( GL_PROJECTION_MATRIX, pj );
 
	/* Get the current MODELVIEW matrix from OpenGL */
	glGetFloatv( GL_MODELVIEW_MATRIX, mv );
	
	m_intrinsic = mat4(pj);
	m_extrinsic = mat4(mv);
 
	/* Combine the two matrices (multiply projection by modelview) */
	clip[ 0] = mv[ 0] * pj[ 0] + mv[ 1] * pj[ 4] + mv[ 2] * pj[ 8] + mv[ 3] * pj[12];
	clip[ 1] = mv[ 0] * pj[ 1] + mv[ 1] * pj[ 5] + mv[ 2] * pj[ 9] + mv[ 3] * pj[13];
	clip[ 2] = mv[ 0] * pj[ 2] + mv[ 1] * pj[ 6] + mv[ 2] * pj[10] + mv[ 3] * pj[14];
	clip[ 3] = mv[ 0] * pj[ 3] + mv[ 1] * pj[ 7] + mv[ 2] * pj[11] + mv[ 3] * pj[15];
 
	clip[ 4] = mv[ 4] * pj[ 0] + mv[ 5] * pj[ 4] + mv[ 6] * pj[ 8] + mv[ 7] * pj[12];
	clip[ 5] = mv[ 4] * pj[ 1] + mv[ 5] * pj[ 5] + mv[ 6] * pj[ 9] + mv[ 7] * pj[13];
	clip[ 6] = mv[ 4] * pj[ 2] + mv[ 5] * pj[ 6] + mv[ 6] * pj[10] + mv[ 7] * pj[14];
	clip[ 7] = mv[ 4] * pj[ 3] + mv[ 5] * pj[ 7] + mv[ 6] * pj[11] + mv[ 7] * pj[15];
 
	clip[ 8] = mv[ 8] * pj[ 0] + mv[ 9] * pj[ 4] + mv[10] * pj[ 8] + mv[11] * pj[12];
	clip[ 9] = mv[ 8] * pj[ 1] + mv[ 9] * pj[ 5] + mv[10] * pj[ 9] + mv[11] * pj[13];
	clip[10] = mv[ 8] * pj[ 2] + mv[ 9] * pj[ 6] + mv[10] * pj[10] + mv[11] * pj[14];
	clip[11] = mv[ 8] * pj[ 3] + mv[ 9] * pj[ 7] + mv[10] * pj[11] + mv[11] * pj[15];
 
	clip[12] = mv[12] * pj[ 0] + mv[13] * pj[ 4] + mv[14] * pj[ 8] + mv[15] * pj[12];
	clip[13] = mv[12] * pj[ 1] + mv[13] * pj[ 5] + mv[14] * pj[ 9] + mv[15] * pj[13];
	clip[14] = mv[12] * pj[ 2] + mv[13] * pj[ 6] + mv[14] * pj[10] + mv[15] * pj[14];
	clip[15] = mv[12] * pj[ 3] + mv[13] * pj[ 7] + mv[14] * pj[11] + mv[15] * pj[15];
 
	/* Extract the numbers for the RIGHT plane */
	frustum[0][0] = clip[ 3] - clip[ 0];
	frustum[0][1] = clip[ 7] - clip[ 4];
	frustum[0][2] = clip[11] - clip[ 8];
	frustum[0][3] = clip[15] - clip[12];
 
	/* Normalize the result */
	t = sqrt( frustum[0][0] * frustum[0][0] + frustum[0][1] * frustum[0][1] + frustum[0][2] * frustum[0][2] );
	frustum[0][0] /= t;
	frustum[0][1] /= t;
	frustum[0][2] /= t;
	frustum[0][3] /= t;
 
	/* Extract the numbers for the LEFT plane */
	frustum[1][0] = clip[ 3] + clip[ 0];
	frustum[1][1] = clip[ 7] + clip[ 4];
	frustum[1][2] = clip[11] + clip[ 8];
	frustum[1][3] = clip[15] + clip[12];
 
	/* Normalize the result */
	t = sqrt( frustum[1][0] * frustum[1][0] + frustum[1][1] * frustum[1][1] + frustum[1][2] * frustum[1][2] );
	frustum[1][0] /= t;
	frustum[1][1] /= t;
	frustum[1][2] /= t;
	frustum[1][3] /= t;
 
	/* Extract the BOTTOM plane */
	frustum[2][0] = clip[ 3] + clip[ 1];
	frustum[2][1] = clip[ 7] + clip[ 5];
	frustum[2][2] = clip[11] + clip[ 9];
	frustum[2][3] = clip[15] + clip[13];
 
	/* Normalize the result */
	t = sqrt( frustum[2][0] * frustum[2][0] + frustum[2][1] * frustum[2][1] + frustum[2][2] * frustum[2][2] );
	frustum[2][0] /= t;
	frustum[2][1] /= t;
	frustum[2][2] /= t;
	frustum[2][3] /= t;
 
	/* Extract the TOP plane */
	frustum[3][0] = clip[ 3] - clip[ 1];
	frustum[3][1] = clip[ 7] - clip[ 5];
	frustum[3][2] = clip[11] - clip[ 9];
	frustum[3][3] = clip[15] - clip[13];
 
	/* Normalize the result */
	t = sqrt( frustum[3][0] * frustum[3][0] + frustum[3][1] * frustum[3][1] + frustum[3][2] * frustum[3][2] );
	frustum[3][0] /= t;
	frustum[3][1] /= t;
	frustum[3][2] /= t;
	frustum[3][3] /= t;
 
	/* Extract the FAR plane */
	frustum[4][0] = clip[ 3] - clip[ 2];
	frustum[4][1] = clip[ 7] - clip[ 6];
	frustum[4][2] = clip[11] - clip[10];
	frustum[4][3] = clip[15] - clip[14];
 
	/* Normalize the result */
	t = sqrt( frustum[4][0] * frustum[4][0] + frustum[4][1] * frustum[4][1] + frustum[4][2] * frustum[4][2] );
	frustum[4][0] /= t;
	frustum[4][1] /= t;
	frustum[4][2] /= t;
	frustum[4][3] /= t;
 
	/* Extract the NEAR plane */
	frustum[5][0] = clip[ 3] + clip[ 2];
	frustum[5][1] = clip[ 7] + clip[ 6];
	frustum[5][2] = clip[11] + clip[10];
	frustum[5][3] = clip[15] + clip[14];
 
	/* Normalize the result */
	t = sqrt( frustum[5][0] * frustum[5][0] + frustum[5][1] * frustum[5][1] + frustum[5][2] * frustum[5][2] );
	frustum[5][0] /= t;
	frustum[5][1] /= t;
	frustum[5][2] /= t;
	frustum[5][3] /= t;
}

bool tgFrustum::PointInFrustum( float x, float y, float z ){
	int p;
 
	for( p = 0; p < 6; p++ ){
		if( frustum[p][0] * x + frustum[p][1] * y + frustum[p][2] * z + frustum[p][3] <= 0 )
			return false;
	}
			
	return true;
}

bool tgFrustum::SphereInFrustum( float x, float y, float z, float radius ){
	int p;
 
	for( p = 0; p < 6; p++ ){
		if( frustum[p][0] * x + frustum[p][1] * y + frustum[p][2] * z + frustum[p][3] <= -radius )
			return false;
	}
	return true;
}

void tgFrustum::DrawFrustum(){
 
// Get near and far from the Projection matrix.
	float depthRange[2];
	glGetFloatv(GL_DEPTH_RANGE, &depthRange[0]);
	float near = -depthRange[0]; //1.0 * m_intrinsic[11] / (m_intrinsic[10] - 1.0);
	float far = -depthRange[1]; //-1.0 * m_intrinsic[11] / (1.0 + m_intrinsic[10]);
 
	// Get the sides of the near plane.
	const float nLeft = near * (m_intrinsic[2] - 1.0f) / m_intrinsic[0];
	const float nRight = near * (1.0f + m_intrinsic[2]) / m_intrinsic[0];
	const float nTop = near * (1.0f + m_intrinsic[6]) / m_intrinsic[5];
	const float nBottom = near * (m_intrinsic[6] - 1.0f) / m_intrinsic[5];
 
	// Get the sides of the far plane.
	const float fLeft = far * (m_intrinsic[2] - 1.0f) / m_intrinsic[0];
	const float fRight = far * (1.0f + m_intrinsic[2]) / m_intrinsic[0];
	const float fTop = far * (1.0f + m_intrinsic[6]) / m_intrinsic[5];
	const float fBottom = far * (m_intrinsic[6] - 1.0f) / m_intrinsic[5];
 
	/*
	 0	glVertex3f(0.0f, 0.0f, 0.0f);
	 1	glVertex3f(nLeft, nBottom, -near);
	 2	glVertex3f(nRight, nBottom, -near);
	 3	glVertex3f(nRight, nTop, -near);
	 4	glVertex3f(nLeft, nTop, -near);
	 5	glVertex3f(fLeft, fBottom, -far);
	 6	glVertex3f(fRight, fBottom, -far);
	 7	glVertex3f(fRight, fTop, -far);
	 8	glVertex3f(fLeft, fTop, -far);
	 */
 	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);


	glMatrixMode(GL_MODELVIEW);

	glPushMatrix();
	glBegin(GL_LINES);
 
		glVertex3f(nLeft, nBottom, -near);
		glVertex3f(fLeft, fBottom, -far);
	
		glVertex3f(nRight, nBottom, -near);
		glVertex3f(fRight, fBottom, -far);
	
		glVertex3f(nRight, nTop, -near);
		glVertex3f(fRight, fTop, -far);
	
		glVertex3f(nLeft, nTop, -near);
		glVertex3f(fLeft, fTop, -far);
	
		//far
		glVertex3f(fLeft, fBottom, -far);
		glVertex3f(fRight, fBottom, -far);
	
		glVertex3f(fRight, fTop, -far);
		glVertex3f(fLeft, fTop, -far);
	
		glVertex3f(fRight, fTop, -far);
		glVertex3f(fRight, fBottom, -far);
	
		glVertex3f(fLeft, fTop, -far);
		glVertex3f(fLeft, fBottom, -far);
	
		//near
		glVertex3f(nLeft, nBottom, -near);
		glVertex3f(nRight, nBottom, -near);
	
		glVertex3f(nRight, nTop, -near);
		glVertex3f(nLeft, nTop, -near);
	
		glVertex3f(nLeft, nTop, -near);
		glVertex3f(nLeft, nBottom, -near);
	
		glVertex3f(nRight, nTop, -near);
		glVertex3f(nRight, nBottom, -near);
	
	glEnd();
	glPopMatrix();
}

