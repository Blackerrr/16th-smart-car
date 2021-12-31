#include "headfile.h"
uint8 left_line[MT9V03X_H], right_line[MT9V03X_H], top_line[MT9V03X_W];
float mid_line[MT9V03X_H]; //����������������ߵ�����
uint8 image_wmid = MT9V03X_W / 2;
int image_hmid = MT9V03X_H / 2;

int goin_cross = 0;
uint32 YZ = 110;
/*****************PID����*************/
//extern _pid servo_pid;
uint32 PWMMID = 790, KP = 100, KD = 0, PROSPECT = 0;
float devia = 0;
float delt;

/****************����״̬����*****************/
uint8 road_status = 0; // 0 ��ʾ��ʼ״̬��1   ��ʾʮ�֣� 2  ��ʾ����·�ڣ�3    ��ʾ���⡣
int sc_status = 1;   // 1 ��ʾ��һ���룬2  ��ʾ��һ�γ���3 ��ʾ�ڶ����룬4 ��ʾ�ڶ��γ�
int judge_status = 1;
/*****************������ر���******************/
int px_num = 0, black_num = 0;
int line_num = 0; // �м��кڵ������
uint8 mu = MISS, md = MISS;
int top_left = 0, top_right = 0;
int bp_y, bp_x; //�ڼ���Ӧ������
/*****************ͻ�����ر���***************/
axis l_d, l_u, r_d, r_u;
axis left_far, right_far, mid_near;
int l_d_bigchange = 0, r_d_bigchange = 0;
int l_u_bigchange = 0, r_u_bigchange = 0;
int Llx_flag = 0, Rlx_flag = 0;
/***********ͼ�����������ֵ��ǰ��������**********/
uint8 gaveValue = 120;
uint8 image_threshold;
uint8 l_IMG[S_img_H][S_img_W];
uint8 r_IMG[S_img_H][S_img_W];
int value_num = 2;
ValueS test_Values;
int tile = 65;
int top_status = 0;
int bla_num = 0;
int valid_num = 0;
int car_line = 45;
/******************************ͼ������************************************/
void Image_Handle(void)
{
    S_Image_zip(mt9v03x_image, l_IMG); //ͼ��ѹ��
    /****************ͼ�����崦��*************/
    value_judge(value_num);//��ֵȷ��
    S_BinaryImage(l_IMG, r_IMG, gaveValue);//Сͼ���ֵ��
    /*************���·ָ�ͼ��ֱ���***********
    test_Values = S_Get_double3S(l_IMG);
    S_BinaryImageS(l_IMG, r_IMG, test_Values);//��ֵ�˲�
    **************��̬ѧ����*****************/
//    S_Open(r_IMG, l_IMG);
//    S_Close(l_IMG, r_IMG);

    /****************ͼ���˲�****************/
    S_WeightedFiltering(r_IMG, l_IMG);//��Ȩ�˲�
    S_Image_larger( l_IMG, mt9v03x_image); //ͼ������
//    get_two(gaveValue);                         //��ֵ��
    edge_detec();       //��Ե����
//    crossroad();        //ʮ���б�
    sc_judge();         //����·���б�
    status_judge();     //����״̬�ж�
    car_judge();
//    get_dif();          //ƫ���ȡ
//seekfree_sendimg_03x(WIRELESS_UART, p, MT9V03X_W, MT9V03X_H);
}
/******************************��ȡ����ͼ��************************************/
void get_image(void)
{
    for (int i = 0; i < MT9V03X_H; i++)
    {
        if (left_line[i] >= 0 && left_line[i] <= MT9V03X_W)
            mt9v03x_image[i][left_line[i]] = white;
        if (right_line[i] >= 0 && right_line[i] <= MT9V03X_W)
            mt9v03x_image[i][right_line[i]] = white;
        if (mid_line[i] >= 0 && mid_line[i] <= MT9V03X_W)
            mt9v03x_image[i][(int)mid_line[i]] = white;
    }
    if(top_status)
    for (int i = TOPleft; i < TOPright; i++)
    {
        if (top_line[i] <= TOP_NEAR && top_line[i] >= mu)
            mt9v03x_image[(int)top_line[i]][i] = white;
    }
}
/******************************ͼ���ֵ��*************************************/
void get_two(uint8 Value)
{
    uint8 *map;
    for (int i = 0; i < MT9V03X_H; i++)
    {
        map = &mt9v03x_image[i][0];
        for (int j = 0; j < MT9V03X_W; j++)
        {
            if (*map > Value)
                *map = DV;
            else
                *map = 0;
            map++;
        }
    }
}
/******************************ͼ��ƫ���ȡ************************************/
void get_dif(void)
{
    int break_flag = 1;
    for(int i = FAR_LINE;i < NEAR_LINE;i++){
        if(left_line[i] != LEFT_LINE || right_line[i] != RIGHT_LINE){
            break_flag = 0;
            if(road_status == 4){
                road_status = 0;
            }
            break;
        }
    }
    if(break_flag){
        road_status = 4;
        break_flag = 0;
    }
}
/******************************б�ʼ���************************************/
double get_K(int y1, int y2, uint8 *x2) //y1 < y2
{
    double sumk = 0;
    uint8 tx2 = *x2;
    x2--;
    for (int i = y2 - 1; i >= y1; i--)
    {
        if ((y2 - i) != 0)
            sumk += (double)(*x2 - tx2) / (y2 - i);
        x2--;
    }
    if (y2 - y1 != 0)
        return sumk / (y2 - y1);
    else
        return 0;
}
/******************************��Сֵ����************************************/
int mmin(int a, int b)
{
    if (a < b)
        return a;
    return b;
}
/******************************���ֵ����********************************/
int mmax(int a, int b)
{
    if (a > b)
        return a;
    return b;
}
/******************************ͬʱ������ֵ����Сֵ����**************************/
void mmax_min(axis *FAR, axis *NEAR, int x, int y)
{
    if (y < FAR->y)
    {
        FAR->y = y;
        FAR->x = x;
        return;
    }
    if (y > NEAR->y)
    {
        NEAR->y = y;
        NEAR->x = x;
    }
}
/******************************��Ե����Ĵ���********************************/
void edge_detection(void)
{
    for (int i = MT9V03X_H - 1; i >= 0; i--) //�߽��ʼ������ȫ����ΪMISS
    {
        left_line[i] = MISS;
        right_line[i] = MISS;
    }
    //ֱ������ʼ�а׵�ƽ��λ��*************
    uint8 *map;
    map = mt9v03x_image[NEAR_LINE]; //mapλ��Ϊimage[NEAR_LINE][0]
    int white_sum = 0;
    int white_num = 0;
    int temp_mid = MT9V03X_W / 2;

    for (int i = LEFT_LINE; i < RIGHT_LINE; i++)
    {
        if ((*map))
        {
            white_sum += i;
            white_num++;
        }
        map++;
    }

    int find_left_edge = 0;
    int find_right_edge = 0;

    if (white_num)
    {
        temp_mid = white_sum / white_num;
        map = &mt9v03x_image[NEAR_LINE][temp_mid];
        for (int i = temp_mid; i > LEFT_LINE; i--) //������������߽�
        {
            if (*map - *(map - 1) == DV)
            {
                find_left_edge = 1;
                left_line[NEAR_LINE] = (uint8)i;
                break;
            }
            map--;
        }

        map = &mt9v03x_image[NEAR_LINE][temp_mid];
        for (int i = temp_mid; i < RIGHT_LINE; i++) //�����������ұ߽�
        {
            if (*map - *(map + 1) == DV)
            {
                find_right_edge = 1;
                right_line[NEAR_LINE] = (uint8)i;
                break;
            }
            map++;
        }
    }
    if (!find_right_edge)
        right_line[NEAR_LINE] = RIGHT_LINE;

    if (!find_left_edge)
        left_line[NEAR_LINE] = LEFT_LINE;
    /*************************************************/
    //��ʼ����*************************
    int current_left_edge = MISS, current_right_edge = MISS, last_edge = 0;

    for (int i = NEAR_LINE - 1; i >= FAR_LINE; i--)
    {
        //����߽�**************************
        current_left_edge = MISS;
        last_edge = left_line[i + 1];
        find_left_edge = 0;

        if (last_edge != MISS)
        {
            if (!mt9v03x_image[i][last_edge]) //��һ��Ϊ�ڣ����ұ���
            {
                map = &mt9v03x_image[i][last_edge];
                for (int j = last_edge; j < RIGHT_LINE; j++)
                {
                    if (*(map + 1) - *map == DV)
                    {
                        current_left_edge = j;
                        find_left_edge = 1;
                        break;
                    }
                    map++;
                }
            }
            else //��һ��Ϊ��
            {
                map = &mt9v03x_image[i][last_edge];
                for (int j = last_edge; j > LEFT_LINE; j--)
                {
                    if (*(map) - *(map - 1) == DV)
                    {
                        current_left_edge = j;
                        find_left_edge = 1;
                        break;
                    }
                    map--;
                }
            }
            if (!find_left_edge) {//ȫ��
                current_left_edge = MISS;
                Llx_flag = i;
            }
        }
        else{
            if (!mt9v03x_image[Llx_flag][last_edge]) //��һ��Ϊ�ڣ����ұ���
            {
                map = &mt9v03x_image[Llx_flag][last_edge];
                for (int j = last_edge; j < RIGHT_LINE; j++)
                {
                    if (*(map + 1) - *map == DV)
                    {
                        current_left_edge = j;
                        find_left_edge = 1;
                        break;
                    }
                    map++;
                }
            }
            else //��һ��Ϊ��
            {
                map = &mt9v03x_image[Llx_flag][last_edge];
                for (int j = last_edge; j > LEFT_LINE; j--)
                {
                    if (*(map) - *(map - 1) == DV)
                    {
                        current_left_edge = j;
                        find_left_edge = 1;
                        break;
                    }
                    map--;
                }
            }
            if (!find_left_edge) //ȫ��
                current_left_edge = MISS;
        }

        //�����ұ߽�****************************
        current_right_edge = MISS;
        last_edge = right_line[i + 1];
        find_right_edge = 0;
        if (last_edge != MISS)
        {
            if (!mt9v03x_image[i][last_edge]) //��һ��Ϊ�� �������
            {
                map = &mt9v03x_image[i][last_edge];
                for (int j = last_edge; j > LEFT_LINE; j--)
                {
                    if (*(map - 1) - *map == DV)
                    {
                        current_right_edge = j;
                        find_right_edge = 1;
                        break;
                    }
                    map--;
                }
            }
            else //��һ��Ϊ�� ���Ҳ���
            {
                map = &mt9v03x_image[i][last_edge];
                for (int j = last_edge; j < RIGHT_LINE; j++)
                {
                    if (*map - *(map + 1) == DV)
                    {
                        current_right_edge = j;
                        find_right_edge = 1;
                        break;
                    }
                    map++;
                }
            }
            if (!find_right_edge) {//ȫ��
                current_right_edge = MISS;
                Rlx_flag = i;
            }
        }
        else{
            if (!mt9v03x_image[Rlx_flag][last_edge]) //��һ��Ϊ�� �������
            {
                map = &mt9v03x_image[Rlx_flag][last_edge];
                for (int j = last_edge; j > LEFT_LINE; j--)
                {
                    if (*(map - 1) - *map == DV)
                    {
                        current_right_edge = j;
                        find_right_edge = 1;
                        break;
                    }
                    map--;
                }
            }
            else //��һ��Ϊ�� ���Ҳ���
            {
                map = &mt9v03x_image[Rlx_flag][last_edge];
                for (int j = last_edge; j < RIGHT_LINE; j++)
                {
                    if (*map - *(map + 1) == DV)
                    {
                        current_right_edge = j;
                        find_right_edge = 1;
                        break;
                    }
                    map++;
                }
            }
            if (!find_right_edge) //ȫ��
                current_right_edge = MISS;
        }
        if (current_left_edge >= current_right_edge)
        { //��߽���ڵ����ұ߽�
            current_left_edge = MISS;
            current_right_edge = MISS;
        }
        left_line[i] = (uint8)current_left_edge;
        right_line[i] = (uint8)current_right_edge;
    }
}
/******************************���ߺϳ�********************************/
void middle_detection(void)
{
    int i;
    if (road_status == 2 && mid_near.y <= 60)
    {
        for (i = FAR_LINE; i < mid_near.y; i++)
        {
            mid_line[i] = image_wmid;
        }
        for (i = mid_near.y; i < NEAR_LINE; i++)
        {
            if (left_line[i] != MISS && right_line[i] != MISS)
            {
                mid_line[i] = (float)(left_line[i] + right_line[i]) / 2;
            }
            else
                mid_line[i] = image_wmid;
        }
    }
    else
    {
        for (i = FAR_LINE; i < NEAR_LINE; i++)
        {
            if (left_line[i] != MISS && right_line[i] != MISS)
            {
                mid_line[i] = (float)(left_line[i] + right_line[i]) / 2;
            }
            else
                mid_line[i] = image_wmid;
        }
    }
}
/******************************����·���ж�********************************/
void sc_judge()
{
    //    int mid;
    mu = MISS;
    md = MISS;
    uint8 i;
    uint8 *map;
    top_status = 0;
    /*******************Ѱ�ҿɿ����Ͻ�*******************/
    for (i = FAR_LINE; i < NEAR_LINE; i++)
    {
        if (left_line[i] != MISS && right_line[i] != MISS)
        {
            mu = i;
            break;
        }
    }
    /*****************Ѱ�Ҵ��ںڵ���½�*****************/
    if (mu > TOP_NEAR)
        return;
    for (i = TOP_NEAR; i > mu; i--)
    { //�������ײ��п�ʼ����Ѱ��
        if (mt9v03x_image[i][image_wmid] == black)
        {
            md = i;
            break;
        }
    }
    /*****************�ж����½��Ƿ���Ч*****************/
    if (md == MISS || md <= mu)
    {
        return;
    }
    else
    {
        top_status = 1;
        get_top();
    } //��ȡ��ͷ��
    /********************Ѱ�������߸�,�����߸�,��***************************/
}

/******************************�ж�ʮ��********************************/

void crossroad()
{
    int maxwhite_sum = 0, len = RIGHT_LINE - LEFT_LINE;
        goin_cross = 0;
        for (int i = NEAR_LINE; i >= FAR_LINE; i--)
            if (right_line[i] - left_line[i] == len)
            {
                maxwhite_sum++;
                if (maxwhite_sum >= 3)
                {
                    goin_cross = 1;
                    break;
                }
            }

        if (!goin_cross)
            return;

        //���¶ο�ʼͻ���
        l_d_bigchange = 0; r_d_bigchange = 0;

        for (int i = NEAR_LINE; i > FAR_LINE; i--)
        {
            if (left_line[i] == MISS || right_line[i] == MISS) //���������ʧ���ҵ���һ����Ч����Ϊͻ���
            {
                continue;
            }
            if (!l_d_bigchange && (left_line[i] - left_line[i - 1] >= 5))
            {
                l_d_bigchange = 1;
                l_d.x = left_line[i];
                l_d.y = i;
            }
            if (!r_d_bigchange && (right_line[i] - right_line[i - 1] <= -5) )
            {
                r_d_bigchange = 1;
                r_d.x = right_line[i];
                r_d.y = i;
            }
            if (l_d_bigchange & r_d_bigchange)
                break;
        }

        //���϶�ͻ���

        l_u_bigchange = 0; r_u_bigchange = 0; //���϶ο�ʼͻ���

        for (int i = FAR_LINE + 10; i < NEAR_LINE - 1; i++)
        {
            if (left_line[i] == MISS || right_line[i] == MISS) //���������ʧ���ҵ���һ����Ч����Ϊͻ���
            {
                continue;
            }
            if (!l_u_bigchange && (left_line[i] - left_line[i + 1] >= 5))
            {
                l_u_bigchange = 1;
                l_u.x = left_line[i];
                l_u.y = i;
            }
            if (!r_u_bigchange && (right_line[i] - right_line[i + 1] <= -5))
            {
                r_u_bigchange = 1;
                r_u.x = right_line[i];
                r_u.y = i;
            }
            if (l_u_bigchange & r_u_bigchange)
                break;
        }

        if (l_d_bigchange && l_u_bigchange && l_u.x <= l_d.x)
            l_u_bigchange = 0;
        if (r_d_bigchange && r_u_bigchange && r_u.x >= r_d.x)
            r_u_bigchange = 0;

        if (l_d_bigchange & r_d_bigchange & (!l_u_bigchange) & (!r_u_bigchange)) //�����¶˶�����ͻ��� �϶�û��
        {

        }
        if (l_u_bigchange & r_u_bigchange & (!l_d_bigchange) & (!r_d_bigchange)) //�����϶˶�����ͻ��� �¶�û��
        {

        }
        else if (l_d_bigchange & r_d_bigchange & l_u_bigchange & r_u_bigchange) //�����¶˶���ͻ��� �϶�Ҳ��
        {
            int cross_num = (l_u.y + l_d.y + r_u.y + r_d.y) / 4;
            if(right_line[cross_num] - left_line[cross_num] > 120){
                road_status = 1;
            }
        }
        else if (l_d_bigchange & l_u_bigchange & (!r_d_bigchange) & (!r_u_bigchange)) //�����������ͻ��� �ұ�û��
        {

        }
        else if (r_d_bigchange & r_u_bigchange & (!l_d_bigchange) & (!l_u_bigchange)) //���ұ�������ͻ��� ���û��
        {

        }
}

/************************���Իع��������б�� y = Ax+B*************************/
float regression(int startline, int endline)
{

    int i = 0, SumX = 0, SumY = 0, SumLines = 0;
    float SumUp = 0, SumDown = 0, avrX = 0, avrY = 0, B, A;
    SumLines = endline - startline; // startline Ϊ��ʼ�У� //endline ������ //SumLines

    for (i = startline; i < endline; i++)
    {
        SumX += i;
        SumY += mid_line[i]; //����mid_lineΪ������ߵ�����
    }
    avrX = SumX / SumLines; //X��ƽ��ֵ
    avrY = SumY / SumLines; //Y��ƽ��ֵ
    SumUp = 0;
    SumDown = 0;
    for (i = startline; i < endline; i++)
    {
        SumUp += (mid_line[i] - avrY) * (i - avrX);
        SumDown += (i - avrX) * (i - avrX);
    }
    if (SumDown == 0)
        B = 0;
    else
        B = (int)(SumUp / SumDown);
    A = (SumY - B * SumX) / SumLines; //�ؾ�
    return B;                         //����б��
}
/****************************������ȷ������********************************/
void edge_detec(void)
{
    for (int i = MT9V03X_H - 1; i >= 0; i--) //�߽��ʼ������ȫ����ΪMISS
    {
        left_line[i] = MISS;
        right_line[i] = MISS;
    }
    //ֱ������ʼ�а׵�ƽ��λ��*************
    uint8 *map;
    map = mt9v03x_image[NEAR_LINE]; //mapλ��Ϊimage[NEAR_LINE][0]
    int white_sum = 0;
    int white_num = 0;
    int temp_mid;

    for (int i = LEFT_LINE; i < RIGHT_LINE; i++)
    {
        if ((*map))
        {
            white_sum += i;
            white_num++;
        }
        map++;
    }

    int find_left_edge = 0;
    int find_right_edge = 0;

    if (white_num)
    {
        temp_mid = white_sum / white_num;
        map = &mt9v03x_image[NEAR_LINE][temp_mid];
        for (int i = temp_mid; i > LEFT_LINE; i--) //������������߽�
        {
            if (*map - *(map - 1) == DV && *(map - 2) == black)
            {
                find_left_edge = 1;
                left_line[NEAR_LINE] = (uint8)i;
                break;
            }
            map--;
        }

        map = &mt9v03x_image[NEAR_LINE][temp_mid];
        for (int i = temp_mid; i < RIGHT_LINE; i++) //�����������ұ߽�
        {
            if (*map - *(map + 1) == DV && *(map + 2) == black)
            {
                find_right_edge = 1;
                right_line[NEAR_LINE] = (uint8)i;
                break;
            }
            map++;
        }
    }
    if (!find_right_edge)
        right_line[NEAR_LINE] = RIGHT_LINE;

    if (!find_left_edge)
        left_line[NEAR_LINE] = LEFT_LINE;
    /*************************************************/
    //��ʼ����*************************
    int current_left_edge = MISS, current_right_edge = MISS, last_edge = 0;

    for (int i = NEAR_LINE - 1; i >= FAR_LINE; i--)
    {
        //����߽�**************************
        current_left_edge = MISS;
        last_edge = left_line[i + 1];
        find_left_edge = 0;

        if (last_edge != MISS)
        {
            if (!mt9v03x_image[i][last_edge]) //��һ��Ϊ�ڣ����ұ���
            {
                map = &mt9v03x_image[i][last_edge];
                for (int j = last_edge; j < RIGHT_LINE; j++)
                {
                    if (*(map + 1) - *map == DV && *(map + 2) == DV)
                    {
                        current_left_edge = j;
                        find_left_edge = 1;
                        break;
                    }
                    map++;
                }

                if (!find_left_edge) //ȫ��
                    Llx_flag = i + 1;
            }
            else //��һ��Ϊ��
            {
                map = &mt9v03x_image[i][last_edge];
                for (int j = last_edge; j > LEFT_LINE; j--)
                {
                    if (*(map) - *(map - 1) == DV && *(map - 2) == black)
                    {
                        current_left_edge = j;
                        find_left_edge = 1;
                        break;
                    }
                    map--;
                }
                if (!find_left_edge) //ȫ��
                    current_left_edge = LEFT_LINE;
            }
        }
        else
        {
            if (!mt9v03x_image[i][Llx_flag]) //��һ��Ϊ�ڣ����ұ���
            {
                map = &mt9v03x_image[i][Llx_flag];
                for (int j = Llx_flag; j < RIGHT_LINE; j++)
                {
                    if (*(map + 1) - *map == DV && *(map + 2) == DV)
                    {
                        current_left_edge = j;
                        find_left_edge = 1;
                        break;
                    }
                    map++;
                }
            }
            else //��һ��Ϊ��
            {
                map = &mt9v03x_image[i][Llx_flag];
                for (int j = Llx_flag; j > LEFT_LINE; j--)
                {
                    if (*(map) - *(map - 1) == DV && *(map - 2) == black)
                    {
                        current_left_edge = j;
                        find_left_edge = 1;
                        break;
                    }
                    map--;
                }
                if (!find_left_edge) //ȫ��
                    current_left_edge = LEFT_LINE;
            }
        }

        //�����ұ߽�****************************
        current_right_edge = MISS;
        last_edge = right_line[i + 1];
        find_right_edge = 0;
        if (last_edge != MISS)
        {
            if (!mt9v03x_image[i][last_edge]) //��һ��Ϊ�� �������
            {
                map = &mt9v03x_image[i][last_edge];
                for (int j = last_edge; j > LEFT_LINE; j--)
                {
                    if (*(map - 1) - *map == DV && *(map - 2) == DV)
                    {
                        current_right_edge = j;
                        find_right_edge = 1;
                        break;
                    }
                    map--;
                }
                if (!find_left_edge) //ȫ��
                    Rlx_flag = i + 1;
            }
            else //��һ��Ϊ�� ���Ҳ���
            {
                map = &mt9v03x_image[i][last_edge];
                for (int j = last_edge; j < RIGHT_LINE; j++)
                {
                    if (*map - *(map + 1) == DV && *(map + 2) == black)
                    {
                        current_right_edge = j;
                        find_right_edge = 1;
                        break;
                    }
                    map++;
                }
                if (!find_right_edge) //ȫ��
                    current_right_edge = RIGHT_LINE;
            }
        }
        else
        {
            if (!mt9v03x_image[i][Rlx_flag]) //��һ��Ϊ�� �������
            {
                map = &mt9v03x_image[i][Rlx_flag];
                for (int j = Rlx_flag; j > LEFT_LINE; j--)
                {
                    if (*(map - 1) - *map == DV && *(map - 2) == DV)
                    {
                        current_right_edge = j;
                        find_right_edge = 1;
                        break;
                    }
                    map--;
                }
            }
            else //��һ��Ϊ�� ���Ҳ���
            {
                map = &mt9v03x_image[i][Rlx_flag];
                for (int j = Rlx_flag; j < RIGHT_LINE; j++)
                {
                    if (*map - *(map + 1) == DV && *(map + 2) == black)
                    {
                        current_right_edge = j;
                        find_right_edge = 1;
                        break;
                    }
                    map++;
                }
                if (!find_right_edge) //ȫ��
                    current_right_edge = RIGHT_LINE;
            }
        }

//        if(current_left_edge == MISS && current_right_edge == MISS)//������ʧ
//            break;
//        left_line[i] = (uint8)current_left_edge;
//        right_line[i] = (uint8)current_right_edge;

        left_line[i] = (uint8)current_left_edge;
        right_line[i] = (uint8)current_right_edge;

        if (current_left_edge == MISS)
        {
            if (current_right_edge == MISS)
                break;
            Llx_flag = i;
        }
        if (current_left_edge == MISS)
        {
            Rlx_flag = i;
        }
    }
}
/******************************״̬�ж�**********************************/
void status_judge(void)
{
    int i;
    uint8 *map;
    left_far.y = MISS;
    right_far.y = MISS;
    mid_near.y = 0;
    for (i = TOPleft; i < image_wmid; i++)
    {
        mmax_min(&left_far, &mid_near, i, top_line[i]);
    }
    for (i = image_wmid; i < TOPright; i++)
    {
        mmax_min(&right_far, &mid_near, i, top_line[i]);
    }
    judge_status = 1;
    if (left_far.x > mid_near.x || right_far.x < mid_near.x || mid_near.y < FAR_LINE + 10)
    {

        road_status = 0;
        judge_status = 0;
        return;
    }
    /*******���������Ǻڵ������µ�ÿ��ƽ���ڵ�*********/
    //�ӳ�������Ϊ���߿�ʼ����ͳ��
    line_num = 0;
    black_num = 0;
    for (i = md; i > mu; i--)
    {
        map = &mt9v03x_image[i][left_line[i]];
        if (left_line[i] == MISS || right_line[i] == MISS)
            break;
        for (int j = left_line[i]; j < right_line[i]; j++)
        {
            if (*map == black)
                black_num++;
            map++;
        }
        line_num++;
    }
    if (line_num != 0)
        px_num = black_num / line_num;
    else
    {
        px_num = 0;
        return;
    }
    /********************Ԥ�ж�����״̬*******************/

    //ʮ��
    if (road_status != 2 && (px_num < 45 && px_num > 5))
    {
        road_status = 1;
    }
    //��������
    else if (black_num < 5 && road_status == 2)
    {
        road_status = 0;
    }
    //����·��
    else if (px_num >= 45 && px_num < 100)
    {
        //if(abs(left_far.y - right_far.y) < 10)
        if(top_line[mid_near.x + 10] - mid_near.y < 10 && top_line[mid_near.x - 10] - mid_near.y < 10)
        road_status = 2;
    }
    else
    {
    }
}
/*****************************��ȡ��ͷ��*****************************/
void get_top(void)
{
    //Ѱ�Ҹ�ͷ��
    uint8 *map;
    int last_edge = 0;
    top_line[image_wmid] = md;
    int find_left_edge = 0, find_right_edge = 0;
    for(int i = LEFT_LINE; i < RIGHT_LINE; i++)
        top_line[i] = TOP_NEAR;
    for (int i = image_wmid - 1; i >= TOPleft; i--)
    {
        //�����ұ߽�**************************
        top_line[i] = TOP_NEAR;
        last_edge = top_line[i + 1];
        find_left_edge = 0;
        if (!mt9v03x_image[last_edge][i]) //�����һ��Ϊ�ڣ�����Ѱ����
        {
            map = &mt9v03x_image[last_edge][i];
            for (int j = last_edge; j <= TOP_NEAR; j++)
            {
                if (*(map + MT9V03X_W) - *map == DV && *(map + 2 * MT9V03X_W) == DV)
                {
                    top_line[i] = j;
                    find_left_edge = 1;
                    break;
                }
                map = map + MT9V03X_W;
            }

            if (!find_left_edge) //ȫ��
                break;
        }
        else //���һ��Ϊ��
        {
            map = &mt9v03x_image[last_edge][i];
            for (int j = last_edge; j > mu; j--)
            {
                if (*(map) - *(map - MT9V03X_W) == DV && *(map - 2 * MT9V03X_W) == black)
                {
                    top_line[i] = j;
                    find_left_edge = 1;
                    break;
                }
                map = map - MT9V03X_W;
            }
            if (!find_left_edge){ //ȫ��
                top_left = i;
                top_line[i] = mu;
            }
        }
    }
    //���ұ��ұ���
    for (int i = image_wmid + 1; i <= TOPright; i++)
    {
        top_line[i] = TOP_NEAR;
        last_edge = top_line[i - 1];
        find_right_edge = 0;
        if (!mt9v03x_image[last_edge][i]) //�ұ���  һ��Ϊ�ڣ�����Ѱ����
        {
            map = &mt9v03x_image[last_edge][i];
            for (int j = last_edge; j < TOP_NEAR; j++)
            {
                if (*(map + MT9V03X_W) - *map == DV && *(map + 2 * MT9V03X_W) == DV)
                {
                    top_line[i] = j;
                    find_right_edge = 1;
                    break;
                }
                map = map + MT9V03X_W;
            }

            if (!find_right_edge) //ȫ��
                break;
        }
        else //��һ��Ϊ��
        {
            map = &mt9v03x_image[last_edge][i];
            for (int j = last_edge; j > mu; j--)
            {
                if (*(map) - *(map - MT9V03X_W) == DV && *(map - 2 * MT9V03X_W) == black)
                {
                    top_line[i] = j;
                    find_right_edge = 1;
                    break;
                }
                map = map - MT9V03X_W;
            }
            if (!find_right_edge){ //ȫ��
                top_right = i;
                top_line[i] = mu;
            }
        }
    }
}
/****************************LCD����*******************************/
void lcd_debug(void)
{
    middle_detection(); //���߻�ȡ
    get_image();        //��ȡ��������ͼ��
/******************����������********************/
    if (l_d_bigchange & l_u_bigchange ) //���������ͻ���
    {
       for(int i = 0; i < MT9V03X_W; i++){
           mt9v03x_image[(l_u.y+l_d.y)/2][i] = MISS;
       }
    }
    else if (r_d_bigchange & r_u_bigchange) //�ұ�������ͻ���
    {
        for(int i = 0; i < MT9V03X_W; i++){
           mt9v03x_image[(r_u.y+r_d.y)/2][i] = MISS;
       }
    }
    lcd_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//    if(road_status == 2)
//    flag.buzzer_flag = 1;
//    lcd_showfloat(0,0,servo_pid.Kp, 2,2);
//    lcd_showfloat(0,1,servo_pid.Kd, 2,2);
//    lcd_showint32(0,2,devia,3);
//    lcd_showfloat(0,0,gave_v.p, 2,2);
//    lcd_showfloat(0,1,gave_v.k, 2,2);
/*********************���ߵ��Բ���*********************/
//    lcd_showint32(60,3,Llx_flag,3);
//    lcd_showint32(60,4,Rlx_flag,3);

/**********************����·�ڵ��Բ���*********************/
//    lcd_showint32(80, 3, NEAR_LINE - bp_y, 3);
//    lcd_showint32(80, 4, bp_x - left_line[NEAR_LINE], 3);
//    lcd_showint32(80,6,black_num,3);
//    lcd_showint32(120, 6, sc_status, 3);//����״̬��ʾ�ڼ���������·��
//    lcd_showint32(80,7,line_num,3);//���ںڵ������������ʾ
        lcd_showint32(80, 3, road_status, 3);
        lcd_showint32(80, 4, px_num, 3);//ƽ�����ߺڵ���
//        lcd_drawpoint(image_wmid, md, BLUE);//���Ľ�����ʾ
        lcd_showint32(80, 5, mid_near.y, 3); //top������ʾ
        lcd_showint32(80, 6, gaveValue, 3);
        lcd_showint32( 0, 1, valid_num, 3);
        lcd_showint32( 0, 2, bla_num, 3);
        lcd_showint32( 0, 3, tri_road_status, 3);
//    lcd_showint32(120, 4, mid_near.x, 3);
/***********************����ͷ��ֵ���Բ���**************
    lcd_showint32(40, 3, gaveValue, 3);
    lcd_showint32(40, 4, S_nums, 3);
    lcd_showint32(40, 5, YZ, 3);
    lcd_showint32(40, 6, tile, 3);
    lcd_showint32(80, 3, value_num, 3);

/***********************ͻ�����Բ���*************************
    if (l_u_bigchange)
    {
        lcd_showint32(0, 3, l_u.y, 3);
        lcd_showint32(40, 3, l_u.x, 3);
    }
    if (l_d_bigchange)
    {
        lcd_showint32(0, 4, l_d.y, 3);
        lcd_showint32(40, 4, l_d.x, 3);
    }
    if (r_u_bigchange)
    {
        lcd_showint32(0, 5, r_u.y, 3);
        lcd_showint32(40, 5, r_u.x, 3);
    }
    if (r_d_bigchange)
    {
        lcd_showint32(0, 6, r_d.y, 3);
        lcd_showint32(40, 6, r_d.x, 3);
    }

////    lcd_drawpoint(l_u.x - DV1,l_u.y,YELLOW);
////    lcd_drawpoint(r_d.x - DV1,r_d.y,YELLOW);
////    lcd_drawpoint(r_u.x - DV1,r_u.y,YELLOW);
//    if(l_d_bigchange)lcd_drawpoint((uint16)l_d.y,(uint16)l_d.x,RED);
//    if(l_u_bigchange)lcd_drawpoint((uint16)l_u.y,(uint16)l_u.x,RED);
//    if(r_d_bigchange)lcd_drawpoint((uint16)r_d.y,(uint16)r_d.x,RED);
//    if(r_u_bigchange)lcd_drawpoint((uint16)r_u.y,(uint16)r_u.x,RED);
///*******************/
}

void value_judge(int value_num){
    switch(value_num){
    case 0:
//        gaveValue = YZ;
        gaveValue = percent(l_IMG,tile);//�ٷֱ�
        break;

    case 1:
//        gaveValue = YZ;
        gaveValue = S_Get_double3(l_IMG);//˫��ȵ�
        break;

    case 2:
//        gaveValue = 88;
        gaveValue = S_GetThresh2(mt9v03x_image);//��ɵ�����
        if(gaveValue == 0){
            gaveValue = percent(l_IMG,tile);
        }
        break;

    case 3:
//        gaveValue = YZ;
        gaveValue = GetMinimumThreshold(l_IMG);//��һ��������
//        gaveValue += 8;
        break;

    case 4:
        gaveValue = YZ;
//        if(GetMinimumThreshold(l_IMG) != -1 ){
//            gaveValue = GetMinimumThreshold(l_IMG);    // ˫�䷨
//        }
//        else
//            gaveValue = YZ;
//        break;

    case 5:
//        gaveValue = YZ;
        gaveValue = S_Get_double2(l_IMG);    //ģ�����
        break;

    case 6:
//        gaveValue = YZ;
        gaveValue = S_Get_01_Value(l_IMG); //��ֵ����
        break;
    }
}

void car_judge(void){
    bla_num = 0;
    valid_num = 0;
    uint8 car_linefar = car_line - 5;
    for(int i = left_line[car_linefar]; i < right_line[car_linefar]; i++){
        if(mt9v03x_image[car_linefar][i] == 0){
            bla_num++;
        }
    }
    if(bla_num > 10){
        for(int i = LEFT_LINE; i < RIGHT_LINE;i++){
            if(mt9v03x_image[car_line][i] - mt9v03x_image[car_line][i+1] == DV && mt9v03x_image[car_line][i+2] == 0){
                valid_num++;
            }
        }
        if(valid_num >= 4){
            road_status = 3;
            return ;
        }

    }
    else{
        car_linefar = 100;
        for(int i = LEFT_LINE; i < RIGHT_LINE;i++){
            if(mt9v03x_image[car_linefar][i] - mt9v03x_image[car_linefar][i+1] == DV && mt9v03x_image[car_linefar][i+2] == 0){
                valid_num++;
            }
        if(valid_num > 4){
            road_status = 3;
        }
        }
    }
}
