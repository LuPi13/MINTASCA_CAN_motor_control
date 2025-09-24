FRACT = 1 << 24  # 2^24

def dec_to_iq24_bytes(value: float) -> str:
    """소수 -> IQ24 32bit(4바이트) HEX 문자열"""
    scaled = int(round(value * FRACT))
    scaled &= 0xFFFFFFFF  # wrap to 32-bit
    b = scaled.to_bytes(4, byteorder='big', signed=False)  # big-endian
    return ' '.join(f'{x:02X}' for x in b)

def iq_bytes_to_dec(hex_str: str) -> float:
    """IQ24 32bit(4바이트) HEX 문자열 -> 소수"""
    # 공백 제거, 대문자/소문자 무시
    s = ''.join(ch for ch in hex_str if ch.isalnum())
    if len(s) % 2 != 0:  # 홀수길이면 0 붙임
        s += '0'
    # 2자리씩 분리
    pairs = [s[i:i+2] for i in range(0, len(s), 2)]
    while len(pairs) < 4:  # 4바이트 안되면 오른쪽 00 채우기
        pairs.append('00')
    first4 = bytes(int(x, 16) for x in pairs[:4])
    val = int.from_bytes(first4, byteorder='big', signed=True)
    return val / FRACT

# 테스트
print(dec_to_iq24_bytes(1.5))   # "01 80 00 00"
print(dec_to_iq24_bytes(-0.25)) # "FF C0 00 00"
print(iq_bytes_to_dec("01 80")) # 1.5
print(iq_bytes_to_dec("FF C0 00 00")) # -0.25