 #include "fuzzy.h"

#define PB 6
#define PM 5
#define PS 4
#define ZO 3
#define NS 2
#define NM 1
#define NB 0

float Fuzzy(float E, float EC)
{
    /*������P����ֵ������*/
    float EFF[7] = {-100, -72, -36, 0, 36, 72, 100};
    /*������D����ֵ������*/
    float DFF[7] = {-6, -3, -1, 0, 1, 3, 6};
    /*�����U����ֵ������(������������ѡ��ͬ�����ֵ)*/
    float UFF[7] = {9, 6, 3, 0, 3, 6, 9};

    //    int rule[7][7]={
    //    //    0  1    2  3    4   5   6
    //        { 6 , 5 , 4 , 3 , 2 , 1 , 0},//0
    //        { 5 , 4 , 3 , 2 , 1 , 0 , 1},//1
    //        { 4 , 3 , 2 , 1 , 0 , 1 , 2},//2
    //        { 3 , 2 , 1 , 0 , 1 , 2 , 3},//3
    //        { 2 , 1 , 0 , 1 , 2 , 3 , 4},//4
    //        { 1 , 0 , 1 , 2 , 3 , 4 , 5},//5
    //        { 0 , 1 , 2 , 3 , 4 , 5 , 6},//6
    //    };

    int rule[7][7] = {
        //    0  1    2  3    4   5   6
        {PB, PB, PM, PM, PS, ZO, ZO}, //0
        {PB, PB, PM, PS, PS, ZO, NS}, //1
        {PB, PM, PM, PS, ZO, NS, NS}, //2
        {PM, PM, PS, ZO, NS, NM, NM}, //3
        {PS, PS, ZO, NS, NS, NM, NB}, //4
        {PS, ZO, NS, NM, NM, NM, NB}, //5
        {ZO, ZO, NM, NM, NM, NB, NB}, //6
    };

    float U = 0;
    /*ƫ��,ƫ��΢���Լ����ֵ�ľ�ȷ��*/
    float PF[2] = {0}, DF[2] = {0}, UF[4] = {0};
    /*ƫ��,ƫ��΢���Լ����ֵ��������*/
    int Pn = 0, Dn = 0, Un[4] = {0};
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;
    /*�����ȵ�ȷ��*/
    /*����PD��ָ������ֵ�����Ч������*/
    if (E > EFF[0] && E < EFF[6])
    {
        if (E <= EFF[1])
        {
            Pn = -2;
            PF[0] = (EFF[1] - E) / (EFF[1] - EFF[0]);
        }
        else if (E <= EFF[2])
        {
            Pn = -1;
            PF[0] = (EFF[2] - E) / (EFF[2] - EFF[1]);
        }
        else if (E <= EFF[3])
        {
            Pn = 0;
            PF[0] = (EFF[3] - E) / (EFF[3] - EFF[2]);
        }
        else if (E <= EFF[4])
        {
            Pn = 1;
            PF[0] = (EFF[4] - E) / (EFF[4] - EFF[3]);
        }
        else if (E <= EFF[5])
        {
            Pn = 2;
            PF[0] = (EFF[5] - E) / (EFF[5] - EFF[4]);
        }
        else if (E <= EFF[6])
        {
            Pn = 3;
            PF[0] = (EFF[6] - E) / (EFF[6] - EFF[5]);
        }
    }

    else if (E <= EFF[0])
    {
        Pn = -2;
        PF[0] = 1;
    }
    else if (E >= EFF[6])
    {
        Pn = 3;
        PF[0] = 0;
    }

    // PF[0]  ��������ߵ�������    PF[1]  �������ұߵ�������
    PF[1] = 1 - PF[0];

    //�ж�D��������
    if (EC > DFF[0] && EC < DFF[6])
    {
        if (EC <= DFF[1])
        {
            Dn = -2;
            DF[0] = (DFF[1] - EC) / (DFF[1] - DFF[0]);
        }
        else if (EC <= DFF[2])
        {
            Dn = -1;
            DF[0] = (DFF[2] - EC) / (DFF[2] - DFF[1]);
        }
        else if (EC <= DFF[3])
        {
            Dn = 0;
            DF[0] = (DFF[3] - EC) / (DFF[3] - DFF[2]);
        }
        else if (EC <= DFF[4])
        {
            Dn = 1;
            DF[0] = (DFF[4] - EC) / (DFF[4] - DFF[3]);
        }
        else if (EC <= DFF[5])
        {
            Dn = 2;
            DF[0] = (DFF[5] - EC) / (DFF[5] - DFF[4]);
        }
        else if (EC <= DFF[6])
        {
            Dn = 3;
            DF[0] = (DFF[6] - EC) / (DFF[6] - DFF[5]);
        }
    }
    //���ڸ�����������
    else if (EC <= DFF[0])
    {
        Dn = -2;
        DF[0] = 1;
    }
    else if (EC >= DFF[6])
    {
        Dn = 3;
        DF[0] = 0;
    }

    // DF[0]  ��������ߵ�������    DF[1]  �������ұߵ�������
    DF[1] = 1 - DF[0];

    /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
    /*���ֵʹ��13����������,����ֵ��UFF[7]ָ��*/
    /*һ�㶼���ĸ�������Ч*/
    Un[0] = rule[Pn + 2][Dn + 2];
    Un[1] = rule[Pn + 3][Dn + 2];
    Un[2] = rule[Pn + 2][Dn + 3];
    Un[3] = rule[Pn + 3][Dn + 3];

    if (PF[0] <= DF[0]) //��С
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];

    /*ͬ���������������ֵ���*/
    if (Un[0] == Un[1])
    {
        if (UF[0] > UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2])
    {
        if (UF[0] > UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3])
    {
        if (UF[0] > UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2])
    {
        if (UF[1] > UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3])
    {
        if (UF[1] > UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3])
    {
        if (UF[2] > UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    t1 = UF[0] * UFF[Un[0]];
    t2 = UF[1] * UFF[Un[1]];
    t3 = UF[2] * UFF[Un[2]];
    t4 = UF[3] * UFF[Un[3]];
    temp1 = t1 + t2 + t3 + t4;
    temp2 = UF[0] + UF[1] + UF[2] + UF[3]; //ģ�������
    U = temp1 / temp2;
    return U;
}
/*******************************************************************************************************************************/
float Fuzzy_D(float E, float EC)
{
    /*������P����ֵ������*/
    float EFF[7] = {-100, -72, -36, 0, 36, 72, 100};
    /*������D����ֵ������*/
    float DFF[7] = {-6, -3, -1, 0, 1, 3, 6};
    /*�����U����ֵ������(������������ѡ��ͬ�����ֵ)*/
    float UFF[7] = {200, 140, 70, 0, 70, 140, 200};

    // int rule[7][7] = {
    //     //    0  1    2  3    4   5   6
    //     {6, 1, 2, 3, 4, 5, 6}, //0
    //     {1, 2, 3, 4, 5, 6, 5}, //1
    //     {2, 3, 4, 5, 6, 5, 4}, //2
    //     {3, 4, 5, 6, 5, 4, 3}, //3
    //     {4, 5, 6, 5, 4, 3, 2}, //4
    //     {5, 6, 5, 4, 3, 2, 1}, //5
    //     {6, 5, 4, 3, 2, 1, 0}, //6
    // };

        int rule[7][7]={
       //    0  1    2  3    4   5   6
           { PB , PB , PM , PM , PS , ZO , ZO},//0
           { PB , PB , PM , PS , PS , ZO , NS},//1
           { PB , PM , PM , PS , ZO , NS , NS},//2
           { PM , PM , PS , ZO , NS , NM , NM},//3
           { PS , PS , ZO , NS , NS , NM , NB},//4
           { PS , ZO , NS , NM , NM , NM , NB},//5
           { ZO , ZO , NM , NM , NM , NB , NB},//6
       };

    float U = 0; /*ƫ��,ƫ��΢���Լ����ֵ�ľ�ȷ��*/
    float PF[2] = {0}, DF[2] = {0}, UF[4] = {0};
    /*ƫ��,ƫ��΢���Լ����ֵ��������*/
    int Pn = 0, Dn = 0, Un[4] = {0};
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;
    /*�����ȵ�ȷ��*/
    /*����PD��ָ������ֵ�����Ч������*/
    if (E > EFF[0] && E < EFF[6])
    {
        if (E <= EFF[1])
        {
            Pn = -2;
            PF[0] = (EFF[1] - E) / (EFF[1] - EFF[0]);
        }
        else if (E <= EFF[2])
        {
            Pn = -1;
            PF[0] = (EFF[2] - E) / (EFF[2] - EFF[1]);
        }
        else if (E <= EFF[3])
        {
            Pn = 0;
            PF[0] = (EFF[3] - E) / (EFF[3] - EFF[2]);
        }
        else if (E <= EFF[4])
        {
            Pn = 1;
            PF[0] = (EFF[4] - E) / (EFF[4] - EFF[3]);
        }
        else if (E <= EFF[5])
        {
            Pn = 2;
            PF[0] = (EFF[5] - E) / (EFF[5] - EFF[4]);
        }
        else if (E <= EFF[6])
        {
            Pn = 3;
            PF[0] = (EFF[6] - E) / (EFF[6] - EFF[5]);
        }
    }

    else if (E <= EFF[0])
    {
        Pn = -2;
        PF[0] = 1;
    }
    else if (E >= EFF[6])
    {
        Pn = 3;
        PF[0] = 0;
    }

    PF[1] = 1 - PF[0];

    //�ж�D��������
    if (EC > DFF[0] && EC < DFF[6])
    {
        if (EC <= DFF[1])
        {
            Dn = -2;
            DF[0] = (DFF[1] - EC) / (DFF[1] - DFF[0]);
        }
        else if (EC <= DFF[2])
        {
            Dn = -1;
            DF[0] = (DFF[2] - EC) / (DFF[2] - DFF[1]);
        }
        else if (EC <= DFF[3])
        {
            Dn = 0;
            DF[0] = (DFF[3] - EC) / (DFF[3] - DFF[2]);
        }
        else if (EC <= DFF[4])
        {
            Dn = 1;
            DF[0] = (DFF[4] - EC) / (DFF[4] - DFF[3]);
        }
        else if (EC <= DFF[5])
        {
            Dn = 2;
            DF[0] = (DFF[5] - EC) / (DFF[5] - DFF[4]);
        }
        else if (EC <= DFF[6])
        {
            Dn = 3;
            DF[0] = (DFF[6] - EC) / (DFF[6] - DFF[5]);
        }
    }
    //���ڸ�����������
    else if (EC <= DFF[0])
    {
        Dn = -2;
        DF[0] = 1;
    }
    else if (EC >= DFF[6])
    {
        Dn = 3;
        DF[0] = 0;
    }

    DF[1] = 1 - DF[0];

    /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
    /*���ֵʹ��13����������,����ֵ��UFF[7]ָ��*/
    /*һ�㶼���ĸ�������Ч*/
    Un[0] = rule[Pn + 2][Dn + 2];
    Un[1] = rule[Pn + 3][Dn + 2];
    Un[2] = rule[Pn + 2][Dn + 3];
    Un[3] = rule[Pn + 3][Dn + 3];

    if (PF[0] <= DF[0]) //��С
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];
    /*ͬ���������������ֵ���*/
    if (Un[0] == Un[1])
    {
        if (UF[0] > UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2])
    {
        if (UF[0] > UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3])
    {
        if (UF[0] > UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2])
    {
        if (UF[1] > UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3])
    {
        if (UF[1] > UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3])
    {
        if (UF[2] > UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    t1 = UF[0] * UFF[Un[0]];
    t2 = UF[1] * UFF[Un[1]];
    t3 = UF[2] * UFF[Un[2]];
    t4 = UF[3] * UFF[Un[3]];
    temp1 = t1 + t2 + t3 + t4;
    temp2 = UF[0] + UF[1] + UF[2] + UF[3]; //ģ�������
    U = temp1 / temp2;
    return U;
}
