 function [outputArg1,outputArg2] = GP(startRecording,stopRecording)
commandStr = 'python cagriGPcodes\slipForecastPython\gp_slipGPyMK.py';
 [status, commandOut] = system(commandStr);
 if status==0
%      fprintf('squared result is %f\n',str2num(commandOut));
% 
disp('GP results are ready');
 end
disp(commandOut);
pause(3)
load cagriGPcodes\slipForecastPython\means.mat means
load cagriGPcodes\slipForecastPython\ses.mat ses
outputArg1 = ses((stopRecording-startRecording)-1:end);
outputArg2 = means((stopRecording-startRecording)-1:end);

end




