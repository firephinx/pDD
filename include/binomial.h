#pragma once

#include <vector>

using namespace std;

struct MultiBinomialSample{
    vector<double> correct;
    vector<double> incorrect;
    
    MultiBinomialSample();
};

struct MultiBinomialInferenceModel{
    vector<double> lp_trial_x_;
    vector<double> lp_ntrial_x_;
    vector<double> lp_trial_nx_;
    vector<double> lp_ntrial_nx_;
    vector<double> lp_x_;
    vector<double> lp_nx_;
    vector<double> independence_;
    vector<double> threshold_;
    
    MultiBinomialInferenceModel();
    MultiBinomialInferenceModel(vector<double> p_trial_x,
                                vector<double> p_trial_nx,
                                vector<double> p_x,
                                vector<double> independence,
                                vector<double> threshold=0.5);
    double PXGivenSample(MultiBinomialSample sample);
    bool Detect(MultiBinomialSample sample);
};

struct BinomialSample{
    double correct;
    double incorrect;
    
    BinomialSample();
};

struct BinomialInferenceModel{
    double lp_trial_x_;
    double lp_ntrial_x_;
    double lp_trial_nx_;
    double lp_ntrial_nx_;
    double lp_x_;
    double lp_nx_;
    double independence_;
    double threshold_;
    
    BinomialInferenceModel();
    BinomialInferenceModel(double p_trial_x,
                           double p_trial_nx,
                           double p_x,
                           double independence=1.0,
                           double threshold=0.5);
    
    double PXGivenSample(BinomialSample sample) const;
    bool Detect(BinomialSample sample) const;
};
