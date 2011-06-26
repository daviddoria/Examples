Imports System.IO

Public Class Form1

    Private Sub btnFileExists_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnFileExists.Click
        Dim File As String = InputBox("Enter a file")
        If System.IO.File.Exists(File) = True Then
            MessageBox.Show("yes")
        Else
            MessageBox.Show("no")
        End If
    End Sub

    Private Sub btnDirectoryExist_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnDirectoryExist.Click
        Dim Directory As String = InputBox("Enter a directory")
        If System.IO.Directory.Exists(Directory) = True Then
            MessageBox.Show("yes")
        Else
            MessageBox.Show("no")
        End If
    End Sub

    Private Sub btnCreateDirectory_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnCreateDirectory.Click
        Dim NewDir As String = InputBox("Enter a directory")
        Directory.CreateDirectory(NewDir)
    End Sub

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        Dim bytes As New Byte()

        Dim fs As New IO.FileStream("c:\test\file.txt", IO.FileMode.Create)
        Dim myWriter As New System.IO.StreamWriter(fs)
        myWriter.WriteLine("hello")

        fs.Close()
    End Sub

    Private Sub btnCopyFile_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnCopyFile.Click
        Dim FileToCopy As String = InputBox("From File?", "From File")
        Dim NewCopy As String = InputBox("To File?", "To File")

        If System.IO.File.Exists(FileToCopy) = True Then
            System.IO.File.Copy(FileToCopy, NewCopy)
            MsgBox("File Copied")
        Else
            MessageBox.Show("File does not exist!")
        End If
    End Sub

    Private Sub btnCopyFolder_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnCopyFolder.Click
        'System.IO.Directory.copy() ' doesn't exist
    End Sub

    Public Function CombineFolderAndFile() As Boolean
        IO.Path.Combine("C:\Hello\", "Goodbye.txt")
        IO.Path.Combine("C:\Hello", "Goodbye.txt")

        'both return "C:\Hello\Goodbye.txt".
        Return True
    End Function

    Private Sub btnZipFolder_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnZipFolder.Click
        'Dim srcfolderString As String = "C:\test"
        'Dim dstfolderString As String = "c:\ZippedFolder.zip"

        ''get folder to zip
        'Dim fbd As New FolderBrowserDialog


        'srcfolderString = fbd.SelectedPath

        ''create empty zip file
        'Dim fileContents() As Byte = {80, 75, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
        'My.Computer.FileSystem.WriteAllBytes(dstfolderString, fileContents, False)

        'Dim objShell As New Shell32.ShellClass
        'Dim objFolderSrc As Shell32.Folder
        'Dim objFolderDst As Shell32.Folder
        'Dim objFolderItems As Shell32.FolderItems

        'objFolderSrc = objShell.NameSpace(srcfolderString)
        'objFolderDst = objShell.NameSpace(dstfolderString)
        'objFolderItems = objFolderSrc.Items
        'objFolderDst.CopyHere(objFolderItems, 20)

        '-------------------------
        'System.IO.Compression.
    End Sub

    Private Sub btnSelectFile_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnSelectFile.Click
        ' If fbd.ShowDialog() = Windows.Forms.DialogResult.OK Then

        'End If


    End Sub
End Class
