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
        Me.btnSendEmail = New System.Windows.Forms.Button
        Me.btnTestPOP = New System.Windows.Forms.Button
        Me.btnMAC = New System.Windows.Forms.Button
        Me.btnReset = New System.Windows.Forms.Button
        Me.btnEmailKK = New System.Windows.Forms.Button
        Me.btnEmailComcast = New System.Windows.Forms.Button
        Me.SuspendLayout()
        '
        'btnSendEmail
        '
        Me.btnSendEmail.Location = New System.Drawing.Point(21, 74)
        Me.btnSendEmail.Name = "btnSendEmail"
        Me.btnSendEmail.Size = New System.Drawing.Size(75, 37)
        Me.btnSendEmail.TabIndex = 0
        Me.btnSendEmail.Text = "Send Email (gmail)"
        Me.btnSendEmail.UseVisualStyleBackColor = True
        '
        'btnTestPOP
        '
        Me.btnTestPOP.Location = New System.Drawing.Point(21, 130)
        Me.btnTestPOP.Name = "btnTestPOP"
        Me.btnTestPOP.Size = New System.Drawing.Size(75, 23)
        Me.btnTestPOP.TabIndex = 1
        Me.btnTestPOP.Text = "Test POP"
        Me.btnTestPOP.UseVisualStyleBackColor = True
        '
        'btnMAC
        '
        Me.btnMAC.Location = New System.Drawing.Point(89, 206)
        Me.btnMAC.Name = "btnMAC"
        Me.btnMAC.Size = New System.Drawing.Size(75, 23)
        Me.btnMAC.TabIndex = 2
        Me.btnMAC.Text = "MAC"
        Me.btnMAC.UseVisualStyleBackColor = True
        '
        'btnReset
        '
        Me.btnReset.Location = New System.Drawing.Point(118, 130)
        Me.btnReset.Name = "btnReset"
        Me.btnReset.Size = New System.Drawing.Size(75, 23)
        Me.btnReset.TabIndex = 3
        Me.btnReset.Text = "Reset"
        Me.btnReset.UseVisualStyleBackColor = True
        '
        'btnEmailKK
        '
        Me.btnEmailKK.Location = New System.Drawing.Point(118, 74)
        Me.btnEmailKK.Name = "btnEmailKK"
        Me.btnEmailKK.Size = New System.Drawing.Size(75, 37)
        Me.btnEmailKK.TabIndex = 4
        Me.btnEmailKK.Text = "Send Email (KK)"
        Me.btnEmailKK.UseVisualStyleBackColor = True
        '
        'btnEmailComcast
        '
        Me.btnEmailComcast.Location = New System.Drawing.Point(217, 74)
        Me.btnEmailComcast.Name = "btnEmailComcast"
        Me.btnEmailComcast.Size = New System.Drawing.Size(75, 37)
        Me.btnEmailComcast.TabIndex = 5
        Me.btnEmailComcast.Text = "Send Email (Comcast)"
        Me.btnEmailComcast.UseVisualStyleBackColor = True
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(312, 266)
        Me.Controls.Add(Me.btnEmailComcast)
        Me.Controls.Add(Me.btnEmailKK)
        Me.Controls.Add(Me.btnReset)
        Me.Controls.Add(Me.btnMAC)
        Me.Controls.Add(Me.btnTestPOP)
        Me.Controls.Add(Me.btnSendEmail)
        Me.Name = "Form1"
        Me.Text = "Form1"
        Me.ResumeLayout(False)

    End Sub
    Friend WithEvents btnSendEmail As System.Windows.Forms.Button
    Friend WithEvents btnTestPOP As System.Windows.Forms.Button
    Friend WithEvents btnMAC As System.Windows.Forms.Button
    Friend WithEvents btnReset As System.Windows.Forms.Button
    Friend WithEvents btnEmailKK As System.Windows.Forms.Button
    Friend WithEvents btnEmailComcast As System.Windows.Forms.Button

End Class
