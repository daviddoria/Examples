Module Module1
    Public Function GetVersionNumber() As Date
        ' Dim ver As String = Me.GetType.Assembly.GetName.Version.ToString
        Dim ver As String = Form1.GetType.Assembly.GetName.Version.ToString

        Dim strStrings() As String = ver.Split(Convert.ToChar("."))

        Dim Days As Integer = Convert.ToInt16(strStrings(2))

        Dim Start As New Date(2000, 1, 1)
        Return Start.AddDays(Days)
    End Function
End Module
