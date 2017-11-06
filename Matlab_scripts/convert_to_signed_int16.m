function [ signed_int16 ] = convert_to_signed_int16( unsigned_int16 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    if (unsigned_int16 > intmax('int16'))
        signed_int16 = unsigned_int16 + 2 * intmin('int16');
    else
        signed_int16 = unsigned_int16;
    end

end

