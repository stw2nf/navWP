# DedroneStriker.spec
# -*- mode: python ; coding: utf-8 -*-

block_cipher = None

a = Analysis(
    ['DedroneStriker.py'],
    pathex=['C:\\path\\to\\your\\source\\files'],
    binaries=[],
    datas=[
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\message_definitions\\v1.0\\ardupilotmega.xml', 'pymavlink/message_definitions/v1.0/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\message_definitions\\v1.0\\common.xml', 'pymavlink/message_definitions/v1.0/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\all.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\ardupilotmega.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\ASLUAV.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\AVSSUAS.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\common.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\csAirLink.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\cubepilot.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\development.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\icarous.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\loweheiser.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\matrixpilot.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\minimal.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\paparazzi.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\python_array_test.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\standard.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\storm32.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\test.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\ualberta.xml', 'pymavlink/dialects/v20/'),
        ('C:\\Python312\\Lib\\site-packages\\pymavlink\\dialects\\v20\\uAvionix.xml', 'pymavlink/dialects/v20/')
        # Add any other XML files needed here
    ],
    hiddenimports=['pymavlink.dialects.v20'],
    hookspath=[],
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='DedroneStriker',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
    # icon='path/to/your/icon.ico'  # Uncomment if you have an icon
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='DedroneStriker'
)
