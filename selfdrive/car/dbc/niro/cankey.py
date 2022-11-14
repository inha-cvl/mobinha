CCAN_filters = [
    # {"can_id": 127,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 304,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 320,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 352,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 356,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 903,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1042,  "can_mask" : 0xfff, "extended": False},
    
    # {"can_id": 1040,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1078,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1151,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1157,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1173,  "can_mask" : 0xfff, "extended": False}, #no
    # {"can_id": 1191,  "can_mask" : 0xfff, "extended": False},# ?
    # {"can_id": 1312,  "can_mask" : 0xfff, "extended": False}, # ?

    # {"can_id": 1280,  "can_mask" : 0xfff, "extended": False}, 
    # {"can_id": 1322,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1342,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1348,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1355,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1369,  "can_mask" : 0xfff, "extended": False},



    # {"can_id": 1427,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1419,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1430,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1426,  "can_mask" : 0xfff, "extended": False},

    # {"can_id": 1535,  "can_mask" : 0xfff, "extended": False},
    # {"can_id": 1225,  "can_mask" : 0xfff, "extended": False}, #not exist in dbc
    # {"can_id": 1291,  "can_mask" : 0xfff, "extended": False}, #not exist in dbc
    # {"can_id": 1294,  "can_mask" : 0xfff, "extended": False}, #not exist in dbc
    # {"can_id": 1429,  "can_mask" : 0xfff, "extended": False}, #not exist in dbc
    # {"can_id": 1470,  "can_mask" : 0xfff, "extended": False}, #not exist in dbc
    # {"can_id": 1473,  "can_mask" : 0xfff, "extended": False}, #not exist in dbc
    # {"can_id": 1507,  "can_mask" : 0xfff, "extended": False}, #not exist in dbc

    {"can_id": 897,  "can_mask" : 0xfff, "extended": False}, #MDPS11
    {"can_id": 1407,  "can_mask" : 0xfff, "extended": False},
    {"can_id": 339,  "can_mask" : 0xfff, "extended": False}, #!!!!!
    {"can_id": 916,  "can_mask" : 0xfff, "extended": False}, #!!!!!
    {"can_id": 902,  "can_mask" : 0xfff, "extended": False}, #!!!!!
    {"can_id": 544,  "can_mask" : 0xfff, "extended": False}, #ok
    {"can_id": 593,  "can_mask" : 0xfff, "extended": False}, #ok
    {"can_id": 688,  "can_mask" : 0xfff, "extended": False}, #ok
    {"can_id": 832,  "can_mask" : 0xfff, "extended": False}, #ok
    {"can_id": 881,  "can_mask" : 0xfff, "extended": False},# ?
    {"can_id": 882,  "can_mask" : 0xfff, "extended": False},
    {"can_id": 1136,  "can_mask" : 0xfff, "extended": False},
    {"can_id": 1168,  "can_mask" : 0xfff, "extended": False}, #0k

    {"can_id": 1265,  "can_mask" : 0xfff, "extended": False}, #CLU 11

    {"can_id": 1287,  "can_mask" : 0xfff, "extended": False}, #ok
    {"can_id": 1345,  "can_mask" : 0xfff, "extended": False}, #!!!!
    {"can_id": 1292,  "can_mask" : 0xfff, "extended": False}, #ok
    {"can_id": 1363,  "can_mask" : 0xfff, "extended": False}, #ok
    {"can_id": 1456,  "can_mask" : 0xfff, "extended": False}, #ok
    #for vans
    {"can_id": 790,  "can_mask" : 0xfff, "extended": False},
]

SCC_filters = [
    {"can_id": 905,  "can_mask" : 0xfff, "extended": False},
    {"can_id": 909,  "can_mask" : 0xfff, "extended": False},
    {"can_id": 1056,  "can_mask" : 0xfff, "extended": False},
    {"can_id": 1057,  "can_mask" : 0xfff, "extended": False},
    {"can_id": 1186,  "can_mask" : 0xfff, "extended": False},
    {"can_id": 1290,  "can_mask" : 0xfff, "extended": False}
]

key = {
        'SCC11' : 1056,
        'SCC12' : 1057,
        'SCC13' : 1290,
        'SCC14' : 905,
        'FCA11' : 909,
        'FRT_RADAR11' : 1186,
        'DCT11' : 128,
        'LFAHDA_MFC' : 1157
                }

ckey = {
    'MDPS12' : 593,
    'CGW2' : 1363,
    'CLU14' : 1301,
    'LKAS11' : 832,
    'CLU11' : 1265,
    'CLU12' : 1456,
    'SAS11' : 688,
    'TCS13' : 916,
    'EMS19' : 1170,
    'EPB11' : 1168,
    'WHL_SPD11' : 902,
    'EMS16' : 608,
    'TCS11' : 339,
    'EMS15' : 1351,
    'CGW1' : 1345,
    'EMS12' : 809,
    'ESP12' : 544,
    'EMS11' : 790,
    'TCU13' : 275,
    'TCU12' : 274,
    'TCU11' : 273,
    'CLU13' : 1292,
    'TCS15' : 1287,
    'P_STS' : 1136,
    'SPAS11' : 912,
    'SPAS12' : 1268,
    'LFAHDA_MFC' : 1157
    }


something = {
        'SCC11' : 0,
        'SCC12' : 0,
        'SCC13' : 0,
        'SCC14' : 0,
        'FCA11' : 0,
        'FRT_RADAR11' : 0,
        'DCT11' : 0
    }

past = {
	'1056' : 0,
	'1057' : 0,
	'1290' : 0,
	'905' : 0,
	'909' : 0,
	'1186' : 0,
	'128' : 0
}

time = {
	'1056' : 100,
	'1057' : 0,
	'1290' : 0,
	'905' : 0,
	'909' : 0,
	'1186' : 0,
	'128' : 0
}

CCAN2SCC = {
    '1363' : 'CGW2',
    '1369' : 'CGW4',
    '593' : 'MDPS12',
    '1155' : 'FCA12',
    '1157' : 'LFAHDA_MFC',
    '1301' : 'CLU14',
    '832' : 'LKAS11',
    '1265' : 'CLU11',
    '1456' : 'CLU12',
    '688' : 'SAS11',
    '916' : 'TCS13',
    '1170' : 'EMS19',
    '1168' : 'EPB11',
    '902' : 'WHL_SPD11',
    '608' : 'EMS16',
    '339' : 'TCS11',
    '1351' : 'EMS15',
    '1345' : 'CGW1',
    '809' : 'EMS12',
    '544' : 'ESP12',
    '790' : 'EMS11',
    '275' : 'TCU13',
    '274' : 'TCU12',
    '273' : 'TCU11',
    '1292' : 'CLU13',
    '1287' : 'TCS15',
    '1136' : 'P_STS'
}

_SCC = {
    'TCU11' : '273',
    'TCU12' : '274',
    'TCU13' : '275',
    'TCS11' : '339',
    'ESP12' : '544',
    'MDPS12' : '593',
    'EMS16' : '608',
    'SAS11' : '688',
    'EMS11' : '790',
    'EMS12' : '809',
    'LKAS11' : '832',
    'WHL_SPD11' : '902',
    'TCS13' : '916',
    'P_STS' : '1136',
    'FCA12' : '1155',
    'LFAHDA_MFC' : '1157',
    'EPB11' : '1168',
    'EMS19' : '1170',
    'CLU11' : '1265',
    'TCS15' : '1287',
    'CLU13' : '1292',
    'CLU14' : '1301',
    'CGW1' : '1345',
    'EMS15' : '1351',
    'CGW2' : '1363',
    'CGW4' : '1369',
    'CLU12' : '1456'
}
