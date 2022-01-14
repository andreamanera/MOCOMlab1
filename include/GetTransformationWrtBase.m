%% GetTransformatioWrtBase function
% inputs:
% - biTei: vector of matrices containing the transformation matrices of link <i> w.r.t. link <i-1> for the current q.
% - linkNumber: for which computing the transformation matrix;
% output:
% - bTi: transformation matrix from the manipulator base to the ith joint in the configuration identified by biTei.

function [bTi] = GetTransformationWrtBase(biTei, linkNumber)

    % initialize the matrix with the transformation matrix from frame 1 wrt 0
    bTi= biTei(:, :, 1);  

    % multiply bTi by the n-th element of the vector biTei
    for n = 2:linkNumber
        bTi = bTi*biTei(:,:,n); 
    end
end