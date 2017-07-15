/****************************/
/*Author: Clarence Chen*/
/*Date: 2017-7-2*/
/****************************/
/*!!!!!!!!READ THIS BEFORE USE!!!!!!!!*/
/*Discription: a library to extract feature vector for points
 * among other points and calculate minimum distance to match
 * two vectors. Rotation is considered. Well tested functions
 * for users: p_feature_extraction, p_feature_sdistance,
 * p_feature_calculate. Use p_feature_calculate to preextract
 * all points feature vectors. The sequence of resulting vector
 * is saved with the order of input points. Use p_feature_sdistance
 * to calculate distance between vectors for matching. Use
 * p_feature_extraction extract feature vector for a particular
 * point, like a point in the image. Specific usage note can be
 * found before each function.*/

#ifndef _POINT_FEATURE
#define _POINT_FEATURE

#include <math.h>
#include <vector>

#define _USE_MATH_DEFINES

using namespace std;

/*extract point feature vector, the first input point is the target feature point, the rest is to extract feature vector*/
int p_feature_extraction(vector<float>& points_x,  vector<float>& points_y, unsigned int dimension, vector<float> &feature_vector)
{
    /*initialize*/
    if(feature_vector.size() != dimension)
    {
        feature_vector.resize(dimension, 0.0);
    }

    /*jump out if the input is invalid*/
    if(points_x.size() != points_y.size() || points_x.size() < 2) return -1;


    /*vector calculate*/
    for(unsigned int i = 1; i < points_x.size(); i++)
    {
        float delt_x = *(points_x.begin() + i) - *points_x.begin();
        float delt_y = *(points_y.begin() + i) - *points_y.begin();

        float dist = sqrt(delt_x*delt_x + delt_y*delt_y);
        float angle = atan2(delt_y, delt_x);
        if(angle < 0) angle += 2*M_PI;

        float peice_angle = 2*M_PI / (float)dimension;
        int index = (int)(angle / peice_angle);

        *(feature_vector.begin() + index) += dist;
    }

    /*normalize*/
    float max_val = 0;
    for(unsigned int i = 0; i < dimension; i ++)
    {
        if(*(feature_vector.begin() + i) > max_val) max_val =  *(feature_vector.begin() + i);
    }

    for(unsigned int i = 0; i < dimension; i ++)
    {
        *(feature_vector.begin() + i) = *(feature_vector.begin() + i) / max_val;
    }

    return 0;
}

/*basic vector distance calculation function*/
float p_feature_sdistance(vector<float> &v1,  vector<float> &v2)
{
    int size = v1.size();
    float sdist = 0;
    for(int i = 0; i <size; i++)
    {
        sdist += (*(v1.begin() + i) - *(v2.begin() + i)) * (*(v1.begin() + i) - *(v2.begin() + i));
    }
    return sdist;
}


/*move vector one step*/
void vector_move_once(vector<float> &v, bool right)
{
    int size = v.size();

    if(right)
    {
        float temp_value = v[0];
        for(int i = 1; i < size; i++)
        {
            float tt = v[i];
            v[i] = temp_value;
            temp_value = tt;
        }
        v[0] = temp_value;
    }
    else
    {
        float temp_value = v[size - 1];
        for(int i = size - 2; i >= 0; i--)
        {
            float tt = v[i];
            v[i] = temp_value;
            temp_value = tt;
        }
        v[size - 1] = temp_value;
    }
}

/*move vector, the last one would be the first one or conversely. move right when times > 0, left when times < 0*/
void vector_move(vector<float> &v, int times)
{
    if(times > 0)
    {
        for(int i = 0; i < times; i++)
        {
            vector_move_once(v, true);
        }
    }
    else if(times < 0 )
    {
        for(int i = 0; i < times; i++)
        {
            vector_move_once(v, false);
        }
    }
}

/*vector distance, with given limitation of rotation rotation in degree(0-180, positive), angle is positive anti-clockwise,return distance*/
/*note: v1 is the premeasured vector*/
/*note: angle error is positive anti-clockwise, error = real angle - set angle*/
float p_feature_sdistance(vector<float> &v1,  vector<float> &v2, float limitation, float &angle)
{
    int size = v1.size();
    int rotate_times = (int)(limitation / 360.0 * size);

    float sdist_zero_angle = p_feature_sdistance(v1, v2);
    float match_sdist = sdist_zero_angle;
    angle = 0.0;

    if(rotate_times > 0)
    {
        /*positive*/
        vector<float> v_p = v1;
        for(int i = 0; i < rotate_times; i++)
        {
            vector_move_once(v_p, true);
            float p_sdist = p_feature_sdistance(v_p, v2);
            if(p_sdist < match_sdist)
            {
                match_sdist = p_sdist;
                angle = (i + 1)*360.0/size;
            }
        }

        /*negative*/
        vector<float> v_n = v1;
        for(int i = 0; i < rotate_times; i++)
        {
            vector_move_once(v_n, false);
            float n_sdist = p_feature_sdistance(v_n, v2);

            if(n_sdist < match_sdist)
            {
                match_sdist = n_sdist;
                angle = 0.0-(i + 1)*360.0/size;
            }
        }
    }


    return match_sdist;
}

/*calculate feature vector for each input point, threshold is the maximum distance of related points in calculation*/
int p_feature_calculate(vector<float> &points_x, vector<float> &points_y, float threshold, unsigned int dimension, vector<vector<float> > &feature_vectors)
{
    int num = points_x.size();
    for(int i = 0; i < num; i++)
    {
        feature_vectors.push_back(vector<float>(dimension,0.0));
    }

    float sthreshold = threshold * threshold;

    for(int i = 0; i < num; i++)
    {
        vector<float> temp_p_x;
        vector<float> temp_p_y;
        temp_p_x.push_back(points_x[i]);
        temp_p_y.push_back(points_y[i]);

        for(int j = 0; j < num; j++)
        {
            if(j != i && ((points_x[j] - points_x[i])*(points_x[j] - points_x[i]) + (points_y[j] - points_y[i])*(points_y[j] - points_y[i]))<sthreshold )
            {
                temp_p_x.push_back(points_x[j]);
                temp_p_y.push_back(points_y[j]);
            }
        }
        p_feature_extraction(temp_p_x, temp_p_y, dimension, feature_vectors[i]);
    }

    return 0;
}


#endif
