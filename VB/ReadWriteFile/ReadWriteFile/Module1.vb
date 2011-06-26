
Module Module1
    Public Function GetFileContents(ByVal FullPath As String, _
       Optional ByRef ErrInfo As String = "") As String

        Dim strContents As String
        Dim objReader As IO.StreamReader
        Try
            objReader = New IO.StreamReader(FullPath)
            strContents = objReader.ReadToEnd()
            objReader.Close()
            Return strContents
        Catch Ex As Exception
            ErrInfo = Ex.Message
            Return ""
        End Try
    End Function

    Function GetAllLines(ByVal FullPath As String) As List(Of String)
        Dim sErr As String = ""
        Dim sContents As String
        sContents = GetFileContents(FullPath, sErr)
        If sErr = "" Then
            Debug.WriteLine("Got file!")
        Else
            Debug.WriteLine("Error retrieving file: " & sErr)
        End If

        Dim AllLines As New List(Of String)

        'works properly, removes the newline chars
        Dim arrRows As String() = Split(sContents, vbNewLine)
        For i As Integer = 0 To arrRows.GetUpperBound(0)
            Dim temp As String = Split(arrRows(i), " ")(0)
            'MessageBox.Show(Split(arrRows(i), " ")(0))
            AllLines.Add(temp)

        Next

        Return AllLines
    End Function

    Public Function SaveTextToFile(ByVal strData As String, _
     ByVal FullPath As String, _
       Optional ByVal ErrInfo As String = "") As Boolean

        Dim bAns As Boolean = False
        Dim objReader As IO.StreamWriter
        Try
            objReader = New IO.StreamWriter(FullPath)
            objReader.Write(strData)
            objReader.Close()
            bAns = True
        Catch Ex As Exception
            ErrInfo = Ex.Message

        End Try
        Return bAns
    End Function

    Public Sub AppendToFile(ByVal strData As String, _
            ByVal FullPath As String)

        Dim objReader As IO.StreamWriter

        objReader = New IO.StreamWriter(FullPath, True)
        objReader.Write(strData)
        objReader.Close()
        
    End Sub

End Module
