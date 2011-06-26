<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> _
Partial Class Form1
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()> _
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        Try
            If disposing AndAlso components IsNot Nothing Then
                components.Dispose()
            End If
        Finally
            MyBase.Dispose(disposing)
        End Try
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    <System.Diagnostics.DebuggerStepThrough()> _
    Private Sub InitializeComponent()
        Me.btnFileExists = New System.Windows.Forms.Button
        Me.btnDirectoryExist = New System.Windows.Forms.Button
        Me.btnCreateDirectory = New System.Windows.Forms.Button
        Me.Button1 = New System.Windows.Forms.Button
        Me.btnCopyFile = New System.Windows.Forms.Button
        Me.btnCopyFolder = New System.Windows.Forms.Button
        Me.btnZipFolder = New System.Windows.Forms.Button
        Me.btnSelectFile = New System.Windows.Forms.Button
        Me.SuspendLayout()
        '
        'btnFileExists
        '
        Me.btnFileExists.Location = New System.Drawing.Point(26, 12)
        Me.btnFileExists.Name = "btnFileExists"
        Me.btnFileExists.Size = New System.Drawing.Size(75, 23)
        Me.btnFileExists.TabIndex = 0
        Me.btnFileExists.Text = "File Exists?"
        Me.btnFileExists.UseVisualStyleBackColor = True
        '
        'btnDirectoryExist
        '
        Me.btnDirectoryExist.Location = New System.Drawing.Point(26, 41)
        Me.btnDirectoryExist.Name = "btnDirectoryExist"
        Me.btnDirectoryExist.Size = New System.Drawing.Size(75, 46)
        Me.btnDirectoryExist.TabIndex = 1
        Me.btnDirectoryExist.Text = "Directory Exists?"
        Me.btnDirectoryExist.UseVisualStyleBackColor = True
        '
        'btnCreateDirectory
        '
        Me.btnCreateDirectory.Location = New System.Drawing.Point(26, 106)
        Me.btnCreateDirectory.Name = "btnCreateDirectory"
        Me.btnCreateDirectory.Size = New System.Drawing.Size(75, 45)
        Me.btnCreateDirectory.TabIndex = 2
        Me.btnCreateDirectory.Text = "Create Directory"
        Me.btnCreateDirectory.UseVisualStyleBackColor = True
        '
        'Button1
        '
        Me.Button1.Location = New System.Drawing.Point(73, 177)
        Me.Button1.Name = "Button1"
        Me.Button1.Size = New System.Drawing.Size(75, 23)
        Me.Button1.TabIndex = 3
        Me.Button1.Text = "Button1"
        Me.Button1.UseVisualStyleBackColor = True
        '
        'btnCopyFile
        '
        Me.btnCopyFile.Location = New System.Drawing.Point(120, 12)
        Me.btnCopyFile.Name = "btnCopyFile"
        Me.btnCopyFile.Size = New System.Drawing.Size(75, 23)
        Me.btnCopyFile.TabIndex = 4
        Me.btnCopyFile.Text = "Copy a Flie"
        Me.btnCopyFile.UseVisualStyleBackColor = True
        '
        'btnCopyFolder
        '
        Me.btnCopyFolder.Location = New System.Drawing.Point(120, 41)
        Me.btnCopyFolder.Name = "btnCopyFolder"
        Me.btnCopyFolder.Size = New System.Drawing.Size(75, 46)
        Me.btnCopyFolder.TabIndex = 5
        Me.btnCopyFolder.Text = "Copy a Folder"
        Me.btnCopyFolder.UseVisualStyleBackColor = True
        '
        'btnZipFolder
        '
        Me.btnZipFolder.Location = New System.Drawing.Point(120, 106)
        Me.btnZipFolder.Name = "btnZipFolder"
        Me.btnZipFolder.Size = New System.Drawing.Size(75, 23)
        Me.btnZipFolder.TabIndex = 6
        Me.btnZipFolder.Text = "Zip Folder"
        Me.btnZipFolder.UseVisualStyleBackColor = True
        '
        'btnSelectFile
        '
        Me.btnSelectFile.Location = New System.Drawing.Point(120, 135)
        Me.btnSelectFile.Name = "btnSelectFile"
        Me.btnSelectFile.Size = New System.Drawing.Size(75, 23)
        Me.btnSelectFile.TabIndex = 7
        Me.btnSelectFile.Text = "Select a File"
        Me.btnSelectFile.UseVisualStyleBackColor = True
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(292, 266)
        Me.Controls.Add(Me.btnSelectFile)
        Me.Controls.Add(Me.btnZipFolder)
        Me.Controls.Add(Me.btnCopyFolder)
        Me.Controls.Add(Me.btnCopyFile)
        Me.Controls.Add(Me.Button1)
        Me.Controls.Add(Me.btnCreateDirectory)
        Me.Controls.Add(Me.btnDirectoryExist)
        Me.Controls.Add(Me.btnFileExists)
        Me.Name = "Form1"
        Me.Text = "Form1"
        Me.ResumeLayout(False)

    End Sub
    Friend WithEvents btnFileExists As System.Windows.Forms.Button
    Friend WithEvents btnDirectoryExist As System.Windows.Forms.Button
    Friend WithEvents btnCreateDirectory As System.Windows.Forms.Button
    Friend WithEvents Button1 As System.Windows.Forms.Button
    Friend WithEvents btnCopyFile As System.Windows.Forms.Button
    Friend WithEvents btnCopyFolder As System.Windows.Forms.Button
    Friend WithEvents btnZipFolder As System.Windows.Forms.Button
    Friend WithEvents btnSelectFile As System.Windows.Forms.Button

End Class
