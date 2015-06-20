clc;
javaaddpath('Weka-3-6\weka.jar');

import weka.core.Instances.*;
import weka.core.converters.ConverterUtils.DataSource.*;
import weka.classifiers.Classifier.*;
import weka.classifiers.Evaluation.*;

%load SVM model
Classifier cls = (Classifier) weka.core.SerializationHelper.read("iPhone_model.model");

DataSource source = new DataSource("features_iPhone_balanced.arff");
Instances data = source.getDataSet();

data.setClassIndex(data.numAttributes() - 1);

Evaluation eval=new Evaluation(data);
prediction=eval.evaluateModel(cls, data);
System.out.println(eval.toSummaryString('\nResults\n======\n',false));
