clc;
javaaddpath('C:\Program Files (x86)\Weka-3-6\weka.jar');

import weka.core.Instances.*;
import weka.core.converters.ConverterUtils.DataSource.*;
import weka.classifiers.Classifier.*;
import weka.classifiers.Evaluation.*;

%load SVM model
Classifier cls = (Classifier) weka.core.SerializationHelper.read("C:/Users/Balajee/Desktop/FM Survey/Accelerometer Data/9 Sep/iPhone_model.model");

DataSource source = new DataSource("C:/Users/Balajee/Desktop/FM Survey/Accelerometer Data/9 Sep/features_iPhone_balanced.arff");
Instances data = source.getDataSet();

data.setClassIndex(data.numAttributes() - 1);

Evaluation eval=new Evaluation(data);
prediction=eval.evaluateModel(cls, data);
System.out.println(eval.toSummaryString('\nResults\n======\n',false));
