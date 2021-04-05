//
// Copyright(c) 2017 by Nobuo NAKAGAWA @ Polyphony Digital Inc.
//
// We"re Hiring!
// http://www.polyphony.co.jp/recruit/
//
#include <cstdlib>
#include <list>
#include <map>
#include <iostream>

#if defined(WIN32)
#include <GL/glut.h>
#ifndef _DEBUG
#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")
#endif // _DEBUG
#elif defined(__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#endif // MACOSX

#include <vector>
#include <string>
#include "glm/glm.hpp"

enum eMode {
  eModePBD,
  eModeXPBD_Concrete,
  eModeXPBD_Wood,
  eModeXPBD_Leather,
  eModeXPBD_Tendon,
  eModeXPBD_Rubber,
  eModeXPBD_Muscle,
  eModeXPBD_Fat,
  eModeMax,
};

static const char* MODE_STRING[eModeMax] = {
  "PBD",
  "XPBD(Concrete)",
  "XPBD(Wood)",
  "XPBD(Leather)",
  "XPBD(Tendon)",
  "XPBD(Rubber)",
  "XPBD(Muscle)",
  "XPBD(Fat)",
};

static const float MODE_COMPLIANCE[eModeMax] = {
  0.0f,            // Miles Macklin"s blog (http://blog.mmacklin.com/2016/10/12/xpbd-slides-and-stiffness/)
  0.00000000004f, // 0.04 x 10^(-9) (M^2/N) Concrete
  0.00000000016f, // 0.16 x 10^(-9) (M^2/N) Wood
  0.000000001f,   // 1.0  x 10^(-8) (M^2/N) Leather
  0.000000002f,   // 0.2  x 10^(-7) (M^2/N) Tendon
  0.0000001f,     // 1.0  x 10^(-6) (M^2/N) Rubber
  0.00002f,       // 0.2  x 10^(-3) (M^2/N) Muscle
  0.0001f,        // 1.0  x 10^(-3) (M^2/N) Fat
};

class CParticle{
 private:
  GLfloat   m_InvMass;
  glm::vec3 m_Position;
  glm::vec3 m_OldPosition;
  glm::vec3 m_Acceleration;

public:
  CParticle(GLfloat inv_mass, glm::vec3& position, glm::vec3& acceleration) :
  m_InvMass(inv_mass),
  m_Position(position),
  m_OldPosition(position),
  m_Acceleration(acceleration){}
  CParticle(){};
  ~CParticle(){}

  void       Update(float t){
    if (m_InvMass > 0.0f){
      glm::vec3 tmp = m_Position;
      m_Position += (m_Position - m_OldPosition) + m_Acceleration * t * t;
      m_OldPosition = tmp;
    }
  }
  glm::vec3& GetPosition()  { return m_Position; }
  GLfloat&   GetInvMass()   { return m_InvMass;  }
  void       AddPosition(const glm::vec3& pos, bool is_force = true){
    if ((m_InvMass > 0.0f) || (is_force)) {
      m_Position += pos;
    }
  }
};

class CApplication{
private:
  float m_Time;
  int   m_SolveTime;
public:
  int   m_IterationNum;
  int   m_Mode;
  int   m_OldMode;
  CApplication() :
  m_Time(0.0f), m_SolveTime(0), m_IterationNum(20), m_Mode(eModePBD), m_OldMode(eModeMax){}

  float GetTime(){ return m_Time; }
  void  SetTime(float time){ m_Time = time; }
  int   GetSolveTime(){ return m_SolveTime; }
  void  SetSolveTime(float time){ m_SolveTime = time; }
};

class CConstraint{
private:
  GLfloat    m_RestLength;
  CParticle* m_Particle1;
  CParticle* m_Particle2;
  GLfloat    m_Stiffness;   // for  PBD(0.0f-1.0f)
  GLfloat    m_Compliance;  // for XPBD
  GLfloat    m_Lambda;      // for XPBD
public:
  CConstraint(CParticle* p0, CParticle* p1) :
  m_RestLength(0.0f),
  m_Particle1(p0),
  m_Particle2(p1),
  m_Stiffness(0.1f),
  m_Compliance(0.0f),
  m_Lambda(0.0f) {
    glm::vec3 p0_to_p1 = m_Particle2->GetPosition() - m_Particle1->GetPosition();
    m_RestLength = glm::length(p0_to_p1);
  }

  void LambdaInit() {
    m_Lambda = 0.0f; // reset every time frame
  }
  void Solve(CApplication& app, float dt){
    GLfloat   inv_mass1         = m_Particle1->GetInvMass();
    GLfloat   inv_mass2         = m_Particle2->GetInvMass();
    GLfloat   sum_mass          = inv_mass1 + inv_mass2;
    if (sum_mass == 0.0f) { return; }
    glm::vec3 p1_minus_p2       = m_Particle1->GetPosition() - m_Particle2->GetPosition();
    GLfloat   distance          = glm::length(p1_minus_p2);
    GLfloat   constraint        = distance - m_RestLength; // Cj(x)
    glm::vec3 correction_vector;
    if (app.m_Mode != eModePBD) { // XPBD
      m_Compliance = MODE_COMPLIANCE[app.m_Mode];
      m_Compliance /= dt * dt;    // a~
      GLfloat dlambda           = (-constraint - m_Compliance * m_Lambda) / (sum_mass + m_Compliance); // eq.18
              correction_vector = dlambda * p1_minus_p2 / (distance + FLT_EPSILON);                    // eq.17
      m_Lambda += dlambda;
    } else {                      // normal PBD
              correction_vector = m_Stiffness * glm::normalize(p1_minus_p2) * -constraint/ sum_mass;   // eq. 1
    }
    m_Particle1->AddPosition(+inv_mass1 * correction_vector);
    m_Particle2->AddPosition(-inv_mass2 * correction_vector);
  }
};

class CBall{
private:
  float     m_Frequency;
  glm::vec3 m_Position;
  float     m_Radius;

public:
  CBall(float radius, float x, float y, float z) :
  m_Frequency(3.14f * 0.4f),
  m_Position(x, y, z),
  m_Radius(radius){}

  void Update(float dt){
    // m_Position.z = cos(m_Frequency) * 2.0f;
    // m_Frequency += dt / 5.0f;
    // if (m_Frequency > 3.14f * 2.0f){ m_Frequency -= 3.14f * 2.0f; }
  }

  void Render(){
    glTranslatef(m_Position.x, m_Position.y, m_Position.z);
    static const glm::vec3 color(0.0f, 0.0f, 1.0f);
    glColor3fv((GLfloat*)&color);
    glutSolidSphere(m_Radius, 30, 30);
  }

  glm::vec3& GetPosition(){ return m_Position; }
  float      GetRadius()  { return m_Radius;   }
};

class CCloth{
private:
  int                      m_Width;
  int                      m_Height;
  std::vector<CParticle>   m_Particles;
  std::vector<CConstraint> m_Constraints;
  
  CParticle* GetParticle(int w, int h) {return &m_Particles[ h * m_Width + w ];}
  void       MakeConstraint(CParticle* p1, CParticle* p2) { m_Constraints.push_back(CConstraint(p1, p2));}

  void DrawTriangle(CParticle* p1, CParticle* p2, CParticle* p3, const glm::vec3 color){
    glColor3fv((GLfloat*)&color);
    glVertex3fv((GLfloat*)&(p1->GetPosition()));
    glVertex3fv((GLfloat*)&(p2->GetPosition()));
    glVertex3fv((GLfloat*)&(p3->GetPosition()));
  }

public:
  CCloth(float width, float height, int num_width, int num_height):
  m_Width(num_height),
  m_Height(num_height) {
    m_Particles.resize(m_Width * m_Height);
    for(int w = 0; w < m_Width; w++){
      for(int h = 0; h < m_Height; h++){
        glm::vec3 pos( width  * ((float)w/(float)m_Width ) - width  * 0.5f,
                      0.3f,
                      -height * ((float)h/(float)m_Height) + height * 0.5f);
        glm::vec3 gravity( 0.0f, -0.8f, 0.0f );
        GLfloat inv_mass = 0.1f;
        // if ((h == 0) && (w == 0)          ||
        //     (h == 0) && (w == m_Width - 1)) {
        //   inv_mass = 0.0f; //fix only edge point
        // }
        m_Particles[ h * m_Width + w ] = CParticle(inv_mass, pos, gravity);
      }
    }
    for(int w = 0; w < m_Width; w++){
      for(int h = 0; h < m_Height; h++){           // structual constraint
        if (w < m_Width  - 1){ MakeConstraint(GetParticle(w, h), GetParticle(w+1, h  )); }
        if (h < m_Height - 1){ MakeConstraint(GetParticle(w, h), GetParticle(w,   h+1)); }
        if (w < m_Width  - 1 && h < m_Height - 1){ // shear constraint
          MakeConstraint(GetParticle(w,   h), GetParticle(w+1, h+1));
          MakeConstraint(GetParticle(w+1, h), GetParticle(w,   h+1));
        }
      }
    }
    for(int w = 0; w < m_Width; w++){
      for(int h = 0; h < m_Height; h++){           // bend constraint
        if (w < m_Width  - 2){ MakeConstraint(GetParticle(w, h), GetParticle(w+2, h  )); }
        if (h < m_Height - 2){ MakeConstraint(GetParticle(w, h), GetParticle(w,   h+2)); }
        if (w < m_Width  - 2 && h < m_Height - 2){
          MakeConstraint(GetParticle(w,   h), GetParticle(w+2, h+2));
          MakeConstraint(GetParticle(w+2, h), GetParticle(w,   h+2));
        }
      }
    }
  }
  ~CCloth(){}

  void Render(){
    glBegin(GL_TRIANGLES);
    int col_idx = 0;
    for(int w = 0; w < m_Width - 1; w++){
      for(int h = 0; h < m_Height - 1; h++){
        glm::vec3 col(1.0f, 0.6f, 0.6f);
        if ( col_idx++ % 2 ){ col = glm::vec3(1.0f, 1.0f, 1.0f);}
        DrawTriangle(GetParticle(w+1,h  ), GetParticle(w,   h), GetParticle(w, h+1), col);
        DrawTriangle(GetParticle(w+1,h+1), GetParticle(w+1, h), GetParticle(w, h+1), col);
      }
    }
    glEnd();
  }

  void Update(CApplication& app, float dt, std::list<CBall>& balls, int iteraion){
    std::vector<CParticle>::iterator particle;
    for(particle = m_Particles.begin(); particle != m_Particles.end(); particle++){
      (*particle).Update(dt); // predict position
    }
    unsigned int  solve_time_ms = 0;
	std::vector<CConstraint>::iterator constraint;
    for(constraint = m_Constraints.begin(); constraint != m_Constraints.end(); constraint++){
      (*constraint).LambdaInit();
    }
    for(int i = 0; i < iteraion; i++){
      for (CBall ball : balls) {
        for(particle = m_Particles.begin(); particle != m_Particles.end(); particle++){
          glm::vec3 vec    = (*particle).GetPosition() - ball.GetPosition();
          float     length = glm::length(vec);
          float     radius = ball.GetRadius() * 1.1; // fake radius
          if (length < radius) {
            (*particle).AddPosition(glm::normalize(vec) * (radius - length));
          }
        }
      }
      unsigned int before = glutGet(GLUT_ELAPSED_TIME);
      for(constraint = m_Constraints.begin(); constraint != m_Constraints.end(); constraint++){
        (*constraint).Solve(app, dt);
      }
      solve_time_ms += glutGet(GLUT_ELAPSED_TIME) - before;
    }
    app.SetSolveTime(solve_time_ms);
  }
};

CApplication g_Application;
CCloth       g_Cloth(1.5f, 1.5f, 40, 40);
std::list<CBall> g_Balls{ 
  CBall(0.1, -0.00964272, 0.00487864, -0.00019705296),
  CBall(0.08, 0.00054020714, -0.095394075, -0.0036351085),
  CBall(0.100999996, 0.15198378, -0.22183715, -0.017068058),
  CBall(0.0845, 0.07931688, -0.18869743, -0.011383),
  CBall(0.068, 0.006649964, -0.1555577, -0.005697942),
  CBall(0.100999996, -0.131352, -0.20754479, -0.043588962),
  CBall(0.0845, -0.062351022, -0.18155125, -0.024643453),
  CBall(0.093399994, 0.2420846, -0.21270585, -0.040538955),
  CBall(0.0866, 0.33278927, -0.23020619, -0.059466075),
  CBall(0.08, 0.42082617, -0.24719185, -0.07783651),
  CBall(0.093399994, -0.21404074, -0.20185287, -0.055183973),
  CBall(0.0866, -0.29637498, -0.22534016, -0.059819847),
  CBall(0.08, -0.37628764, -0.24813664, -0.06431937),
  CBall(0.07339999, 0.50924724, -0.25322366, -0.08937363),
  CBall(0.066599995, 0.60034776, -0.25943828, -0.10126037),
  CBall(0.06, 0.68876886, -0.2654701, -0.1127975),
  CBall(0.07339999, -0.47129363, -0.2560258, -0.07179497),
  CBall(0.066599995, -0.5691786, -0.26415402, -0.079497114),
  CBall(0.06, -0.6641846, -0.2720432, -0.08697271),
  CBall(0.105000004, 0.08969513, -0.3231805, -0.0020129606),
  CBall(0.11, 0.09738652, -0.45480677, 0.02527158),
  CBall(0.114999995, 0.1050779, -0.58643305, 0.05255612),
  CBall(0.12, 0.11276928, -0.7180593, 0.07984066),
  CBall(0.105000004, -0.066214666, -0.3246228, -0.009860255),
  CBall(0.11, -0.07034518, -0.4660233, 0.023834959),
  CBall(0.114999995, -0.07447569, -0.6074238, 0.057530172),
  CBall(0.12, -0.07860621, -0.74882424, 0.091225386),
};

void render_string(std::string& str, int w, int h, int x0, int y0) {
  glDisable(GL_LIGHTING);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, w, h, 0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glRasterPos2f(x0, y0);
  int size = (int)str.size();
  for(int i = 0; i < size; ++i){
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[i]);
  }
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

void init(int argc, char* argv[]){
  // for (const auto& [key, value] : pose_data) {
  //   auto ball = CBall(0.15, value.at("z"), value.at("x"), value.at("y"));
  //   g_Balls.push_back(ball);
  // }

  glClearColor(0.7f, 0.7f, 0.65f, 1.0f);
  glEnable(GL_CULL_FACE);

  GLfloat time = (float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
  g_Application.SetTime(time);
}

void display(void){
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glDepthFunc(GL_LESS); 
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_NORMALIZE);

  glPushMatrix();
    g_Cloth.Render();
  glPopMatrix();

  
  for (CBall g_Ball : g_Balls) {
    glPushMatrix();
    g_Ball.Render();
    glPopMatrix();
  }
  glColor3d(1.0f, 1.0f, 1.0f);
  char debug[128];
  sprintf(debug, "ITERATION %d", g_Application.m_IterationNum);
  std::string iteration_text(debug);
  render_string(iteration_text, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 10, 20);
  sprintf(debug, "%s", MODE_STRING[g_Application.m_Mode]);
  std::string mode_text(debug);
  render_string(mode_text, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 10, 40);
  sprintf(debug, "TIME %d(ms)", g_Application.GetSolveTime());
  std::string time_text(debug);
  render_string(time_text, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), 10, 60);

  glutSwapBuffers();
}

void reshape(int width, int height){
  static GLfloat lightPosition[4] = {0.0f,  2.5f,  5.5f, 1.0f};
  static GLfloat lightDiffuse[3]  = {1.0f,  1.0f,  1.0f      };
  static GLfloat lightAmbient[3]  = {0.25f, 0.25f, 0.25f     };
  static GLfloat lightSpecular[3] = {1.0f,  1.0f,  1.0f      };

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glShadeModel(GL_SMOOTH);

  glViewport(0, 0, width, height);
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(30.0, (double)width / (double)height, 0.0001f, 1000.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(0.0f, 0.0f, 5.0f, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0); // pos, tgt, up

  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  lightDiffuse);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  lightAmbient);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
}

void idle(void){
  GLfloat time = (float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
  GLfloat dt = time - g_Application.GetTime();

  dt = (dt > 0.033f) ? 0.033f : dt; // keep 30fps
  // for (CBall g_Ball : g_Balls) {
  //   g_Ball.Update(dt);
  // }
  g_Cloth.Update(g_Application, dt, g_Balls, g_Application.m_IterationNum);

  g_Application.SetTime(time);
  glutPostRedisplay();
}

void keyboard(unsigned char key , int x , int y){
  switch(key){
  case 27: exit(0); break; // esc
  }
}

void special(int key, int x, int y){
  if (key == GLUT_KEY_UP) {
    g_Application.m_IterationNum++;
  }
  if (key == GLUT_KEY_DOWN) {
    if (g_Application.m_IterationNum > 1){
      g_Application.m_IterationNum--;
    }
  }
  if (key == GLUT_KEY_LEFT) {
    if (g_Application.m_Mode > eModePBD) {
      g_Application.m_OldMode = g_Application.m_Mode;
      g_Application.m_Mode--;
    }
  }
  if (key == GLUT_KEY_RIGHT) {
    if (g_Application.m_Mode < eModeMax - 1) {
      g_Application.m_OldMode = g_Application.m_Mode;
      g_Application.m_Mode++;
    }
  }
}

int main(int argc, char* argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(640, 480);
  glutCreateWindow("XPBD: Position-Based Simulation of Compliant Constrained Dynamics");

  init(argc, argv);

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutIdleFunc(idle);
  glutKeyboardFunc(keyboard);
  glutSpecialFunc(special);

  g_Application.m_OldMode = g_Application.m_Mode;
  g_Application.m_Mode += 7;

  glutMainLoop();
  return 0;
}
