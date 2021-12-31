#include "headfile.h"
uint8 left_line[MT9V03X_H], right_line[MT9V03X_H], top_line[MT9V03X_W];
float mid_line[MT9V03X_H]; //用来存放赛道中心线的数组
uint8 image_wmid = MT9V03X_W / 2;
int image_hmid = MT9V03X_H / 2;

int goin_cross = 0;
uint32 YZ = 110;
/*****************PID变量*************/
//extern _pid servo_pid;
uint32 PWMMID = 790, KP = 100, KD = 0, PROSPECT = 0;
float devia = 0;
float delt;

/****************赛道状态变量*****************/
uint8 road_status = 0; // 0 表示初始状态，1   表示十字， 2  表示三岔路口，3    表示车库。
int sc_status = 1;   // 1 表示第一次入，2  表示第一次出，3 表示第二次入，4 表示第二次出
int judge_status = 1;
/*****************三叉相关变量******************/
int px_num = 0, black_num = 0;
int line_num = 0; // 中间有黑点的行数
uint8 mu = MISS, md = MISS;
int top_left = 0, top_right = 0;
int bp_y, bp_x; //黑尖点对应的坐标
/*****************突变点相关变量***************/
axis l_d, l_u, r_d, r_u;
axis left_far, right_far, mid_near;
int l_d_bigchange = 0, r_d_bigchange = 0;
int l_u_bigchange = 0, r_u_bigchange = 0;
int Llx_flag = 0, Rlx_flag = 0;
/***********图像基本处理，二值化前基本变量**********/
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
/******************************图像处理函数************************************/
void Image_Handle(void)
{
    S_Image_zip(mt9v03x_image, l_IMG); //图像压缩
    /****************图像整体处理*************/
    value_judge(value_num);//阈值确定
    S_BinaryImage(l_IMG, r_IMG, gaveValue);//小图像二值化
    /*************上下分割图像分别处理***********
    test_Values = S_Get_double3S(l_IMG);
    S_BinaryImageS(l_IMG, r_IMG, test_Values);//中值滤波
    **************形态学处理*****************/
//    S_Open(r_IMG, l_IMG);
//    S_Close(l_IMG, r_IMG);

    /****************图像滤波****************/
    S_WeightedFiltering(r_IMG, l_IMG);//加权滤波
    S_Image_larger( l_IMG, mt9v03x_image); //图像扩大
//    get_two(gaveValue);                         //二值化
    edge_detec();       //边缘计算
//    crossroad();        //十字判别
    sc_judge();         //三叉路口判别
    status_judge();     //赛道状态判断
    car_judge();
//    get_dif();          //偏差获取
//seekfree_sendimg_03x(WIRELESS_UART, p, MT9V03X_W, MT9V03X_H);
}
/******************************获取边线图像************************************/
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
/******************************图像二值化*************************************/
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
/******************************图像偏差获取************************************/
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
/******************************斜率计算************************************/
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
/******************************最小值函数************************************/
int mmin(int a, int b)
{
    if (a < b)
        return a;
    return b;
}
/******************************最大值函数********************************/
int mmax(int a, int b)
{
    if (a > b)
        return a;
    return b;
}
/******************************同时获得最大值和最小值函数**************************/
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
/******************************边缘计算的代码********************************/
void edge_detection(void)
{
    for (int i = MT9V03X_H - 1; i >= 0; i--) //边界初始化，先全部赋为MISS
    {
        left_line[i] = MISS;
        right_line[i] = MISS;
    }
    //直接求起始行白点平均位置*************
    uint8 *map;
    map = mt9v03x_image[NEAR_LINE]; //map位置为image[NEAR_LINE][0]
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
        for (int i = temp_mid; i > LEFT_LINE; i--) //从中向左找左边界
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
        for (int i = temp_mid; i < RIGHT_LINE; i++) //从中向右找右边界
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
    //开始爬边*************************
    int current_left_edge = MISS, current_right_edge = MISS, last_edge = 0;

    for (int i = NEAR_LINE - 1; i >= FAR_LINE; i--)
    {
        //找左边界**************************
        current_left_edge = MISS;
        last_edge = left_line[i + 1];
        find_left_edge = 0;

        if (last_edge != MISS)
        {
            if (!mt9v03x_image[i][last_edge]) //下一行为黑，向右边找
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
            else //下一行为白
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
            if (!find_left_edge) {//全白
                current_left_edge = MISS;
                Llx_flag = i;
            }
        }
        else{
            if (!mt9v03x_image[Llx_flag][last_edge]) //下一行为黑，向右边找
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
            else //下一行为白
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
            if (!find_left_edge) //全白
                current_left_edge = MISS;
        }

        //查找右边界****************************
        current_right_edge = MISS;
        last_edge = right_line[i + 1];
        find_right_edge = 0;
        if (last_edge != MISS)
        {
            if (!mt9v03x_image[i][last_edge]) //下一行为黑 向左查找
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
            else //下一行为白 向右查找
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
            if (!find_right_edge) {//全白
                current_right_edge = MISS;
                Rlx_flag = i;
            }
        }
        else{
            if (!mt9v03x_image[Rlx_flag][last_edge]) //下一行为黑 向左查找
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
            else //下一行为白 向右查找
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
            if (!find_right_edge) //全白
                current_right_edge = MISS;
        }
        if (current_left_edge >= current_right_edge)
        { //左边界大于等于右边界
            current_left_edge = MISS;
            current_right_edge = MISS;
        }
        left_line[i] = (uint8)current_left_edge;
        right_line[i] = (uint8)current_right_edge;
    }
}
/******************************中线合成********************************/
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
/******************************三叉路口判断********************************/
void sc_judge()
{
    //    int mid;
    mu = MISS;
    md = MISS;
    uint8 i;
    uint8 *map;
    top_status = 0;
    /*******************寻找可靠的上界*******************/
    for (i = FAR_LINE; i < NEAR_LINE; i++)
    {
        if (left_line[i] != MISS && right_line[i] != MISS)
        {
            mu = i;
            break;
        }
    }
    /*****************寻找存在黑点的下界*****************/
    if (mu > TOP_NEAR)
        return;
    for (i = TOP_NEAR; i > mu; i--)
    { //从赛道底部行开始往上寻找
        if (mt9v03x_image[i][image_wmid] == black)
        {
            md = i;
            break;
        }
    }
    /*****************判断上下界是否有效*****************/
    if (md == MISS || md <= mu)
    {
        return;
    }
    else
    {
        top_status = 1;
        get_top();
    } //获取盖头线
    /********************寻找左上线高,右上线高,低***************************/
}

/******************************判断十字********************************/

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

        //找下段开始突变点
        l_d_bigchange = 0; r_d_bigchange = 0;

        for (int i = NEAR_LINE; i > FAR_LINE; i--)
        {
            if (left_line[i] == MISS || right_line[i] == MISS) //如果赛道消失，找到下一个有效点作为突变点
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

        //找上端突变点

        l_u_bigchange = 0; r_u_bigchange = 0; //从上段开始突变点

        for (int i = FAR_LINE + 10; i < NEAR_LINE - 1; i++)
        {
            if (left_line[i] == MISS || right_line[i] == MISS) //如果赛道消失，找到下一个有效点作为突变点
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

        if (l_d_bigchange & r_d_bigchange & (!l_u_bigchange) & (!r_u_bigchange)) //两边下端都存在突变点 上端没有
        {

        }
        if (l_u_bigchange & r_u_bigchange & (!l_d_bigchange) & (!r_d_bigchange)) //两边上端都存在突变点 下端没有
        {

        }
        else if (l_d_bigchange & r_d_bigchange & l_u_bigchange & r_u_bigchange) //两边下端都有突变点 上端也有
        {
            int cross_num = (l_u.y + l_d.y + r_u.y + r_d.y) / 4;
            if(right_line[cross_num] - left_line[cross_num] > 120){
                road_status = 1;
            }
        }
        else if (l_d_bigchange & l_u_bigchange & (!r_d_bigchange) & (!r_u_bigchange)) //仅左边上下有突变点 右边没有
        {

        }
        else if (r_d_bigchange & r_u_bigchange & (!l_d_bigchange) & (!l_u_bigchange)) //仅右边上下有突变点 左边没有
        {

        }
}

/************************线性回归计算中线斜率 y = Ax+B*************************/
float regression(int startline, int endline)
{

    int i = 0, SumX = 0, SumY = 0, SumLines = 0;
    float SumUp = 0, SumDown = 0, avrX = 0, avrY = 0, B, A;
    SumLines = endline - startline; // startline 为开始行， //endline 结束行 //SumLines

    for (i = startline; i < endline; i++)
    {
        SumX += i;
        SumY += mid_line[i]; //这里mid_line为存放中线的数组
    }
    avrX = SumX / SumLines; //X的平均值
    avrY = SumY / SumLines; //Y的平均值
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
    A = (SumY - B * SumX) / SumLines; //截距
    return B;                         //返回斜率
}
/****************************生长线确定边线********************************/
void edge_detec(void)
{
    for (int i = MT9V03X_H - 1; i >= 0; i--) //边界初始化，先全部赋为MISS
    {
        left_line[i] = MISS;
        right_line[i] = MISS;
    }
    //直接求起始行白点平均位置*************
    uint8 *map;
    map = mt9v03x_image[NEAR_LINE]; //map位置为image[NEAR_LINE][0]
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
        for (int i = temp_mid; i > LEFT_LINE; i--) //从中向左找左边界
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
        for (int i = temp_mid; i < RIGHT_LINE; i++) //从中向右找右边界
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
    //开始爬边*************************
    int current_left_edge = MISS, current_right_edge = MISS, last_edge = 0;

    for (int i = NEAR_LINE - 1; i >= FAR_LINE; i--)
    {
        //找左边界**************************
        current_left_edge = MISS;
        last_edge = left_line[i + 1];
        find_left_edge = 0;

        if (last_edge != MISS)
        {
            if (!mt9v03x_image[i][last_edge]) //下一行为黑，向右边找
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

                if (!find_left_edge) //全黑
                    Llx_flag = i + 1;
            }
            else //下一行为白
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
                if (!find_left_edge) //全白
                    current_left_edge = LEFT_LINE;
            }
        }
        else
        {
            if (!mt9v03x_image[i][Llx_flag]) //下一行为黑，向右边找
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
            else //下一行为白
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
                if (!find_left_edge) //全白
                    current_left_edge = LEFT_LINE;
            }
        }

        //查找右边界****************************
        current_right_edge = MISS;
        last_edge = right_line[i + 1];
        find_right_edge = 0;
        if (last_edge != MISS)
        {
            if (!mt9v03x_image[i][last_edge]) //下一行为黑 向左查找
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
                if (!find_left_edge) //全黑
                    Rlx_flag = i + 1;
            }
            else //下一行为白 向右查找
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
                if (!find_right_edge) //全白
                    current_right_edge = RIGHT_LINE;
            }
        }
        else
        {
            if (!mt9v03x_image[i][Rlx_flag]) //下一行为黑 向左查找
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
            else //下一行为白 向右查找
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
                if (!find_right_edge) //全白
                    current_right_edge = RIGHT_LINE;
            }
        }

//        if(current_left_edge == MISS && current_right_edge == MISS)//赛道消失
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
/******************************状态判断**********************************/
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
    /*******计算中线是黑点的情况下的每行平均黑点*********/
    //从出现中心为黑线开始往上统计
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
    /********************预判断赛道状态*******************/

    //十字
    if (road_status != 2 && (px_num < 45 && px_num > 5))
    {
        road_status = 1;
    }
    //正常赛道
    else if (black_num < 5 && road_status == 2)
    {
        road_status = 0;
    }
    //三叉路口
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
/*****************************获取盖头线*****************************/
void get_top(void)
{
    //寻找盖头线
    uint8 *map;
    int last_edge = 0;
    top_line[image_wmid] = md;
    int find_left_edge = 0, find_right_edge = 0;
    for(int i = LEFT_LINE; i < RIGHT_LINE; i++)
        top_line[i] = TOP_NEAR;
    for (int i = image_wmid - 1; i >= TOPleft; i--)
    {
        //往左找边界**************************
        top_line[i] = TOP_NEAR;
        last_edge = top_line[i + 1];
        find_left_edge = 0;
        if (!mt9v03x_image[last_edge][i]) //左边那一行为黑，向下寻找找
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

            if (!find_left_edge) //全黑
                break;
        }
        else //左边一行为白
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
            if (!find_left_edge){ //全白
                top_left = i;
                top_line[i] = mu;
            }
        }
    }
    //往右边找边线
    for (int i = image_wmid + 1; i <= TOPright; i++)
    {
        top_line[i] = TOP_NEAR;
        last_edge = top_line[i - 1];
        find_right_edge = 0;
        if (!mt9v03x_image[last_edge][i]) //右边那  一行为黑，向下寻找找
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

            if (!find_right_edge) //全黑
                break;
        }
        else //下一行为白
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
            if (!find_right_edge){ //全白
                top_right = i;
                top_line[i] = mu;
            }
        }
    }
}
/****************************LCD调试*******************************/
void lcd_debug(void)
{
    middle_detection(); //中线获取
    get_image();        //获取边线中线图像
/******************基本调试用********************/
    if (l_d_bigchange & l_u_bigchange ) //左边上下有突变点
    {
       for(int i = 0; i < MT9V03X_W; i++){
           mt9v03x_image[(l_u.y+l_d.y)/2][i] = MISS;
       }
    }
    else if (r_d_bigchange & r_u_bigchange) //右边上下有突变点
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
/*********************边线调试部分*********************/
//    lcd_showint32(60,3,Llx_flag,3);
//    lcd_showint32(60,4,Rlx_flag,3);

/**********************三叉路口调试部分*********************/
//    lcd_showint32(80, 3, NEAR_LINE - bp_y, 3);
//    lcd_showint32(80, 4, bp_x - left_line[NEAR_LINE], 3);
//    lcd_showint32(80,6,black_num,3);
//    lcd_showint32(120, 6, sc_status, 3);//三叉状态显示第几次如三叉路口
//    lcd_showint32(80,7,line_num,3);//存在黑点的中线行数显示
        lcd_showint32(80, 3, road_status, 3);
        lcd_showint32(80, 4, px_num, 3);//平均中线黑点数
//        lcd_drawpoint(image_wmid, md, BLUE);//中心近点显示
        lcd_showint32(80, 5, mid_near.y, 3); //top底线显示
        lcd_showint32(80, 6, gaveValue, 3);
        lcd_showint32( 0, 1, valid_num, 3);
        lcd_showint32( 0, 2, bla_num, 3);
        lcd_showint32( 0, 3, tri_road_status, 3);
//    lcd_showint32(120, 4, mid_near.x, 3);
/***********************摄像头阈值调试部分**************
    lcd_showint32(40, 3, gaveValue, 3);
    lcd_showint32(40, 4, S_nums, 3);
    lcd_showint32(40, 5, YZ, 3);
    lcd_showint32(40, 6, tile, 3);
    lcd_showint32(80, 3, value_num, 3);

/***********************突变点调试部分*************************
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
        gaveValue = percent(l_IMG,tile);//百分比
        break;

    case 1:
//        gaveValue = YZ;
        gaveValue = S_Get_double3(l_IMG);//双峰谷底
        break;

    case 2:
//        gaveValue = 88;
        gaveValue = S_GetThresh2(mt9v03x_image);//逐飞迭代法
        if(gaveValue == 0){
            gaveValue = percent(l_IMG,tile);
        }
        break;

    case 3:
//        gaveValue = YZ;
        gaveValue = GetMinimumThreshold(l_IMG);//新一代迭代法
//        gaveValue += 8;
        break;

    case 4:
        gaveValue = YZ;
//        if(GetMinimumThreshold(l_IMG) != -1 ){
//            gaveValue = GetMinimumThreshold(l_IMG);    // 双蜂法
//        }
//        else
//            gaveValue = YZ;
//        break;

    case 5:
//        gaveValue = YZ;
        gaveValue = S_Get_double2(l_IMG);    //模糊大津法
        break;

    case 6:
//        gaveValue = YZ;
        gaveValue = S_Get_01_Value(l_IMG); //均值比例
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
