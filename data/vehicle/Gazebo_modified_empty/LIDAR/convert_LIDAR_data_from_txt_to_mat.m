% function convert_LIDAR_data_from_txt_to_mat(num_of_LIDAR_frames)

num_of_LIDAR_frames= 1537
for i=0:num_of_LIDAR_frames
    
    % read the text file
    fileID= fopen(['textFiles/',num2str(i),'.txt'],'r');
    z= transpose(fscanf(fileID,'%f',[2,inf]));
    fclose(fileID);
    % TODO: this two tranformations will be done in the ROS code directly
    if ~isempty(z)
        z= [z(:,1),z(:,2)]; % for the vehicle dataset
    end
    
    % save as mat files
    save(['matFiles/','Epoch',num2str(i),'.mat'],'z');
end
