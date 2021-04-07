function s = getSizes(varargin)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

nArgin = length(varargin);
s = zeros(nArgin,2);
for idxArgin = 1:nArgin
    s(idxArgin,:) = size(varargin{idxArgin});
end

