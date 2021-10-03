#include "contral.h"
#include "Init.h"
#include "sys.h"  


int height;     //�ò��������Ǳ�����������������ֵ��������������������������˳�����ʵ����
//                ��������̨Ϊ����С�����յ��ĳ����������ź������ͺ���Ҫ�õ��������ݲ���֮��
//                ��Ӧ����Ԥ�⣬��Ҫ�Դ�����ֵ���п������˲�������˵���������˲�������һ������Ԥ�⹦�ܵĵ�ͨ�˲�����
/*
  �������˲����������»���Ӧ�õ�����ͷ׷��Ŀ��ʱ��ϵͳ����̨�Ŀ��ƣ��ײ�Կ��������п������˲�����Ϊ��̨�趨ֵ��������̨��
*/
//float kalman_height=0;



//2. �Ը߶�Ϊ�� ���忨�����ṹ�岢��ʼ������


/**
 *�������˲���
 *@param KFP *kfp �������ṹ�����
 *   float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��

���ڹ���������ֵ����ֵԽС�������Ƕ�ģ��Ԥ��ֵ����Խ�ߣ�ϵͳ������ҲԽ�죬��֮�෴�����ڲ��������ң���ֵԽ��������ǶԲ���ֵ����Խ�ͣ��������ʱϵͳ����Ϊ��Ӧ����
��С������ϵͳ�𵴡���������ʱ�̶�һ������һ������С���������ֵʹϵͳ�����ٶ��������Ӵ�С������ֵʹ�������ӽ���ʵ��
���ڣأ��еĳ�ʼֵ����ֵ�����˿�ʼʱ�������ٶȣ�һ������Ϊ������ֵ��ͬ���������߽�С����������Ͽ�����������ŵ����Ľ��У���ֵ������Ϊ��С�Ĺ���Э�������

 */
 float kalmanFilter(KFP *kfp,float input)
 {
     //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
     kfp->Now_P = kfp->LastP + kfp->Q;
     //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
     //����Э�����: ���ε�ϵͳЭ����� kfp->LastP ����һ������׼����
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }
 