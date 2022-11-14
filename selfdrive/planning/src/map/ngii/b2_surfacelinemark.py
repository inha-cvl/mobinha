class B2_SURFACELINEMARK:
    def __init__(self, b2_surfacelinemark):
        for key in b2_surfacelinemark.keys():
            if key[:2] == 'R_':
                R_LinkID = key
            
            elif key[:2] == 'L_':
                L_LinkID = key

        self.ID = b2_surfacelinemark['ID']
        self.AdminCode = b2_surfacelinemark['AdminCode']
        self.Type = b2_surfacelinemark['Type']
        self.Kind = b2_surfacelinemark['Kind']
        self.R_linkID = b2_surfacelinemark[R_LinkID]
        self.L_linkID = b2_surfacelinemark[L_LinkID]
        self.UpdateDate = b2_surfacelinemark['UpdateDate']
        self.Maker = b2_surfacelinemark['Maker']
        self.Version = b2_surfacelinemark['Version']
        self.Remark = b2_surfacelinemark['Remark']
        self.geometry = b2_surfacelinemark['geometry']