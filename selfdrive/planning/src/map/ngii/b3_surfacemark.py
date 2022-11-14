class B3_SURFACEMARK:
    def __init__(self, b3_surfacemark):
        self.ID = b3_surfacemark['ID']
        self.AdminCode = b3_surfacemark['AdminCode']
        self.Type = b3_surfacemark['Type']
        self.Kind = b3_surfacemark['Kind']
        self.LinkID = b3_surfacemark['LinkID']
        self.UpdateDate = b3_surfacemark['UpdateDate']
        self.Maker = b3_surfacemark['Maker']
        self.Version = b3_surfacemark['Version']
        self.Remark = b3_surfacemark['Remark']
        self.geometry = b3_surfacemark['geometry']