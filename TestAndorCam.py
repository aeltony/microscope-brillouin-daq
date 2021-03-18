from ctypes import *
import time

ERROR_CODE = {
    0: 'AT_SUCCESS',
    1: 'AT_ERR_NOTINITIALISED',
    2: 'AT_ERR_NOTIMPLEMENTED',
    3: 'AT_ERR_READONLY',
    4: 'AT_ERR_NOTREADABLE',
    5: 'AT_ERR_NOTWRITABLE',
    6: 'AT_ERR_OUTOFRANGE',
    7: 'AT_ERR_INDEXNOTAVAILABLE',
    8: 'AT_ERR_INDEXNOTIMPLEMENTED',
    9: 'AT_ERR_EXCEEDEDMAXSTRINGLENGTH',
    10: 'AT_ERR_CONNECTION',
    11: 'AT_ERR_NODATA',
    12: 'AT_ERR_INVALIDHANDLE',
    13: 'AT_ERR_TIMEDOUT',
    14: 'AT_ERR_BUFFERFULL',
    15: 'AT_ERR_INVALIDSIZE',
    16: 'AT_ERR_INVALIDALIGNMENT',
    17: 'AT_ERR_COMM',
    18:'AT_ERR_STRINGNOTAVAILABLE',
    19: 'AT_ERR_STRINGNOTIMPLEMENTED',
    20: 'AT_ERR_NULL_FEATURE',
    21: 'AT_ERR_NULL_HANDLE',
    22: 'AT_ERR_NULL_IMPLEMENTED_VAR',
    23: 'AT_ERR_NULL_READABLE_VAR',
    24: 'AT_ERR_NULL_READONLY_VAR',
    25: 'AT_ERR_NULL_WRITABLE_VAR',
    26: 'AT_ERR_NULL_MINVALUE',
    27: 'AT_ERR_NULL_MAXVALUE',
    28: 'AT_ERR_NULL_VALUE',
    29: 'AT_ERR_NULL_STRING',
    30: 'AT_ERR_NULL_COUNT_VAR',
    31: 'AT_ERR_NULL_ISAVAILABLE_VAR',
    32: 'AT_ERR_NULL_MAXSTRINGLENGTH',
    33: 'AT_ERR_NULL_EVCALLBACK',
    34: 'AT_ERR_NULL_QUEUE_PTR',
    35: 'AT_ERR_NULL_WAIT_PTR',
    36: 'AT_ERR_NULL_PTRSIZE',
    37: 'AT_ERR_NOMEMORY',
    100: 'AT_ERR_HARDWARE_OVERFLOW'
}

if __name__ == '__main__':
    # Load library for Windows
    dll = WinDLL('C:\\Users\\Mandelstam\\Source\Repos\\human-brillouin-daq\\Devices\\Andor_DLL_wrap\\atcore')
    utildll = WinDLL('C:\\Users\\Mandelstam\\Source\Repos\\human-brillouin-daq\\Devices\\Andor_DLL_wrap\\atutility')
    #dll = WinDLL('C:\\Program Files\\Andor SDK3\\examples\\serialnumber\\atcore')
    #utildll = WinDLL('C:\\Program Files\\Andor SDK3\\examples\\serialnumber\\atutility')
    error = dll.AT_InitialiseLibrary('')
    print('Init library error =', ERROR_CODE[error])
    error = utildll.AT_InitialiseUtilityLibrary('')
    print('Init util library error =', ERROR_CODE[error])
    #num_cam = c_longlong()
    #error = dll.AT_GetInt('AT_HANDLE_SYSTEM', 'DeviceCount', byref(num_cam))
    #print('Device count error =', ERROR_CODE[error])
    #print('num_cam =', num_cam.value)
    max_lng = c_int()
    error = dll.AT_GetStringMaxLength('AT_HANDLE_SYSTEM','SoftwareVersion', byref(max_lng))
    if error != 0:
        print('AT_GetStringMaxLength error:', ERROR_CODE[error])
        max_lng = c_int(17)
    lib_ver = create_string_buffer(max_lng.value)
    error = dll.AT_GetString('AT_HANDLE_SYSTEM', 'SoftwareVersion', byref(lib_ver))
    lv = lib_ver.raw
    version = lv.decode('utf-8')
    print('Library version =', version)

    handle = c_longlong()
    error = dll.AT_Open(int(0), byref(handle))
    print('Device open error =', ERROR_CODE[error])
    print('handle =', handle)
