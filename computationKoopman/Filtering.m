%% load all data from Maria's experiments
clear 
close all;
clc 

workspace;  % Make sure the workspace panel is showing.
format longg;
format compact;

addpath 'C:\Users\mamak\Dropbox\Important Documents\Research\Project 3 - Koopman\TailFish\Fish Data - April 2019 (30 Hz  Frame Rate)\2pi - SmoothDataAndFiniteDifferences'
% Define a starting folder.
start_path = fullfile(matlabroot, '\toolbox\images\imdemos');
% Ask user to confirm or change.
% topLevelFolder = uigetdir(start_path);
topLevelFolder = 'C:\Users\mamak\Dropbox\Important Documents\Research\Project 3 - Koopman\TailFish\Fish Data - April 2019 (30 Hz  Frame Rate)\2pi - SmoothDataAndFiniteDifferences';
if topLevelFolder == 0
	return;
end
% Get list of all subfolders.
allSubFolders = genpath(topLevelFolder);
% Parse into a cell array.
remain = allSubFolders;
listOfFolderNames = {};
while true
	[singleSubFolder, remain] = strtok(remain, ';');
	if isempty(singleSubFolder)
		break;
	end
	listOfFolderNames = [listOfFolderNames singleSubFolder];
end
numberOfFolders = length(listOfFolderNames);

clear singleSubFolder remain start_path topLevelFolder allSubFolders

    x_int_list = [];
    y_int_list = [];
    psi_int_list = [];
    v1_int_list = [];
    v2_int_list = [];
    omega_int_list = [];
    omega_a_list = [];
    u1_list = [];
    u2_list = [];
    
    addpath('C:\Users\mamak\Dropbox\Important Documents\Research\Project 3 - Koopman\Sagital Glider');
    tailfish_parameters();
    Lengths = [0]; % length of each data set

for i = 2 : numberOfFolders 
    name = listOfFolderNames{i}
    cd(name)

%     files = dir('*.txt');
    files = dir('STATES*.txt');
    for j=1:length(files)
        eval(['load ' files(j).name ' -ascii']);
    end
    % control parameters
    omega_a = 2*pi;
    numbers_in_folder_name = regexp(name,'-?\d+\.?\d*|-?\d*\.?\d+','match');
    a0 = str2num(numbers_in_folder_name{end});
    
    aa = str2num(numbers_in_folder_name{end-1});
%     a0 = 40;  
    a0 = a0 * pi/180; % [-40, + 40] degrees convert to radians        
    aa = aa * pi/180;
%             close all;

    % In each subfolder, store both trials
    for trial = 1 : 2
        no = num2str(trial);
        currentFolder = cd;
        
        if isfile(['STATES', no, '.txt'])
        
            filename = eval((['STATES', no]));
            t = filename(:,1);
            v1 = filename(:,2);
            v2 = filename(:,3);
            omega = filename(:,4);
            %5th is wrapped angle
            x = filename(:,6);
            y = filename(:,7);
%             psi = filename(:,8);
            psi = filename(:,5);

            
            
            %% Filtering the velocities
             clear v1 v2 omega
            
%              Filter data using a TV-LP filter
             dt = 0.3333;
%              x = x/1000; y = y/1000; % scale from mm to m
            [x, vx_w] = TVplusLPfilter(x, dt);
            [y, vy_w] = TVplusLPfilter(y, dt);
            [psi, omega] = TVplusLPfilter(psi, dt);
            t(end-1:end) = []; t(1:2) = []; 
            
            % Convert world-frame linear velocities to body-frame linear velocities
            
            for jj = 1 : length(x)
                th = psi(jj);
                R = [cos(th), -sin(th); sin(th), cos(th)];
                temp = R\[vx_w(jj); vy_w(jj)];
                v1(jj,1) = temp(1);
                v2(jj,1) = temp(2);
            end
           
%             filename(:,6:7) = filename(:,6:7)/1000; % convert x-y to SI units (from mm to m)
            fid = fopen(files(trial).name, 'wt');
            filename = filename';
%             fprintf(fid, '%f %f %f %f %f %f %f \n', t'; v1'; v2'; omega'; x'; y'; psi');
            filename = [t'; v1'; v2'; omega'; psi'; x'; y'];            
            fprintf(fid, '%f %f %f %f %f %f %f \n', filename(:,1:1:end));
            fclose(fid);
            
            clear x_int y_int psi_int v1_int v2_int omega_int ASTATES1 ASTATES2
            
        end
    end
end
