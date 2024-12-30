#ifndef _ROBOTVAR_H__
#define _ROBOTVAR_H__

#define SYSTEM_OK    0   // ϵͳ����
#define SYSTEM_ERR  -1   // ϵͳ����

#ifndef SYSTEM_TRUE
#define SYSTEM_TRUE  1
#endif
#ifndef SYSTEM_FALSE
#define SYSTEM_FALSE 0
#endif
#define Square(n)   ((n) * (n))              // ƽ��
#define Cube(n)     ((n) * (n) * (n))        // ����

//#define First 1
//#define Second 2
#define RigidBlend  0

#ifndef Max
#define Max(a,b)    (((a) > (b)) ? (a) : (b))
#endif
#define Max3(a,b,c) Max(Max(a,b), (c))
#ifndef Min
#define Min(a,b)    (((a) < (b)) ? (a) : (b))
#endif

#define ROBOT_TYPE_4   4	/*����������--����*/
#define ROBOT_TYPE_6   6	/*����������--����*/
#define SERVO_TYPE     0	/*�ŷ�����*/
#define JOINT_NUM	   6   // �岹����,s
#define POSE_NUM       6
#define DELTA_TAU_MAX  1
// [[ ϵͳ����
#ifndef DOF
#define DOF            9       // ���������ɶ�
#endif

#ifndef TEST_ROBOT_CONTROL
#define INTERP_T	   0.004  // �岹����,s
#define POS_T          0.004     // λ������,s
#else
#define INTERP_T       0.016     // (test)�岹����,s
#define POS_T          0.004     // (test)λ������,s
#endif
#define VerifyError 0.000001
#define RT_LITTLE   0.0000001    // ϵͳ��ʶ�����С����
#define SERR         0.0000001 //0.00001    // �������Χ
#define POS_SERR      0.00036    // �ؽ�λ���������Χ, 1cts - 0.00036(���ٱ�100)
#define POS_PRECISION       1    // ��λ�þ���, �ŷ����
#ifndef PI
#define PI      3.1415926535898  // Բ����
#endif
#define PI2     6.2831853071796  // 2��Բ����  
#define PI_RAD  0.0174532925199  // �Ƕ�ת��Ϊ���Ȳ���
#define PI_DEG  57.2957795130823  // ����ת��Ϊ�ǶȲ���
#define ERR_STOP_T     1//  1.000 //��s
// ]]


// [[ �˶�״̬
#define RUN_MODE_TEACH     1     // ʾ��ģʽ
#define RUN_MODE_AUTO      2     // ����ģʽ
#define RUN_MODE_ERR       0     // ����ģʽ
#define RUN_DIR_POS        1     // ������
#define RUN_DIR_NEG       -1     // ������
#define RUN_STATE_RUNNING  1     // �����˶�
#define RUN_STATE_STOP     0     // ֹͣ
#define RUN_STOPMODE_HOLD  0     // ��ͣ
#define RUN_STOPMODE_STOP  1     // ֹͣ
#define RUN_LIM_POS        1     // ������
#define RUN_LIM_NOT        0     // �޼���
#define RUN_LIM_NEG       -1     // ������
// ]]

// [[ ����ϵ
#define COORDINATE_JOINT   1    // �ؽ�����ϵ
#define COORDINATE_WORLD   2    // ��������ϵ
#define COORDINATE_TOOL    3    // ��������ϵ
#define COORDINATE_USER    4    // �û�����ϵ
#define COORDINATE_ADDITIONALAXIS  5//����������ϵ
#define COORDINATE_JOINT_VEL   1    // �ؽ�����ϵ
#define COORDINATE_WORLD_VEL   2    // ��������ϵ
#define COORDINATE_TOOL_VEL    3    // ��������ϵ
#define COORDINATE_USER_VEL    4    // �û�����ϵ
#define COORDINATE_ADDITIONALAXIS_VEL 5///����������ϵ

#define COORDINATE_JOINT_POS  11    // �ؽ�����ϵ
#define COORDINATE_WORLD_POS  12    // ��������ϵ
#define COORDINATE_TOOL_POS   13    // ��������ϵ
#define COORDINATE_USER_POS   14    // �û�����ϵ
#define COORDINATE_ADDITIONALAXIS_POS 15//����������ϵ

#define MOTOR_ENABLE       1     // ʹ��
#define MOTOR_DISABLE      0     // ��ʹ��


#define  TrajJoint               0
#define  TrajLine                1
#define  TrajBsLine              2  

#define   VelModeSTsmooth        1 
#define   VelModePoly            2  
#define   VelModeSTshape         3 
#define   VelModeSshapeInv       4   
#define   BlendError             0 
#define   BlendJointST           1  
#define   BlendJointPoly         2 
#define   BlendCarSpace          3  
#define   BlendJointFusion       4   
#define   NoneMode               99 
#define   STSecNum               8/*?��???��?��???��???��?????��?y*/
#define   DAxisNum              2*DOF/*������ɶ���*/
#define  VelSecNum              2*DOF
#define  QuatSlerpMode          0 
#define  QuatBezierMode         1  
#define  SignleAngleMode        2 
#define  PolyAngleMode          3  
#define   BSNode0                0.0 
#define   BSNode1                0.366 
#define   BSNode2                0.5 
#define   BSNode3                0.634  
#define   BSNode4                1.0    
#define    LFS_ISTshape          1  
#define    LFS_BlendMode         0    

#define RT_LITTLE   0.0000001
#define PI      3.1415926535898  // Բ����

#define PI2     6.2831853071796  // 2��Բ����  
#define PI_RAD  0.0174532925199  // �Ƕ�ת��Ϊ���Ȳ���
#define PI_DEG  57.2957795130823  // ����ת��Ϊ�ǶȲ���
#define PROG_VELMODE_EULER     1  // ŷ�����ٶ�ģʽ
#define PROG_VELMODE_ANGLE     2  // ���ٶ�ģʽ

#define MAX_OVERFOLW_PULSE  65536
#define MAX_OVERFOLW_DEG    11796480  //(32768 * 360)

#define U16 unsigned short int
#define U32 unsigned  int

#endif
