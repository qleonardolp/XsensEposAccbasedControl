function exotaulog = importfile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  EXOTAU = IMPORTFILE(FILENAME) reads data from text file FILENAME for
%  the default selection.  Returns the numeric data.
%
%  EXOTAU = IMPORTFILE(FILE, DATALINES) reads data for the specified row
%  interval(s) of text file FILENAME. Specify DATALINES as a positive
%  scalar integer or a N-by-2 array of positive scalar integers for
%  dis-contiguous row intervals.
%
%  Example:
%  exotau = importfile("C:\Users\leona\Documents\accbasedcontrol_xsens_epos\ACC_BASED_CONTROL\data\log_thread002.txt", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 16-Feb-2022 16:18:27

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Setup the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 11);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "theta_h", "dtheta_h", "ddtheta_h", "theta_r", "dtheta_r", "ddtheta_r", "torque_sea", "torque_int", "torque_mtr", "dtheta_mtr"];
% opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.VariableTypes = ["single", "single", "single", "single", "single", "single", "single", "single", "single", "single", "single"];


% Specify file level properties
opts.ImportErrorRule = "omitvar";
opts.MissingRule = "omitvar";
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
exotaulog = readtable(filename, opts);

%% Convert to output type
exotaulog = table2array(exotaulog);

% Compare Velocities (deg/s):
plot(exotaulog(:,1), rad2deg(exotaulog(:,3))), hold on
plot(exotaulog(:,1), rad2deg(exotaulog(:,6))), grid on
legend('user','robot')
title('Velocities (deg/s)')
hold off
end