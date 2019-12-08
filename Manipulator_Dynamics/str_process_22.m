function [ss] = str_process_22(s)
s1 = strsplit(s,',');
for i = 1:size(s1,2)
    ss(i) = str2double(s1{i});
end